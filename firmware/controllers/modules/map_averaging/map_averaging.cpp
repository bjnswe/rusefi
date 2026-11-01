/**
 * @file	map_averaging.cpp
 *
 * In order to have best MAP estimate possible, we real MAP value at a relatively high frequency
 * and average the value within a specified angle position window for each cylinder
 *
 * @date Dec 11, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 *
 * This file is part of rusEfi - see http://rusefi.com
 *
 * rusEfi is free software; you can redistribute it and/or modify it under the terms of
 * the GNU General Public License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * rusEfi is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "pch.h"
#include "exp_average.h"
#include "cylinder_pressure_core.h"

#if EFI_PROD_CODE && EFI_CYLINDER_PRESSURE && HAL_USE_ADC && !defined(STM32H7XX)
#include "knock_config.h"
#include "software_knock.h"
#endif


#if EFI_MAP_AVERAGING && defined (MODULE_MAP_AVERAGING)
#if !EFI_SHAFT_POSITION_INPUT
	fail("EFI_SHAFT_POSITION_INPUT required to have EFI_EMULATE_POSITION_SENSORS")
#endif // EFI_SHAFT_POSITION_INPUT

#include "map_averaging.h"
#include "trigger_central.h"

#if EFI_PROD_CODE && EFI_CYLINDER_PRESSURE && HAL_USE_ADC && !defined(STM32H7XX)

// ADC3 runs continuously into a circular DMA buffer. DMA callbacks only publish
// completed half-buffer boundaries; angle interpolation and accumulation happen
// from the regular engine callback, outside interrupt context.
static constexpr size_t PressureDmaSize = SoftwareKnockSampleBufferSize;
static constexpr size_t PressureDmaHalfSize = PressureDmaSize / 2;
static constexpr size_t TriggerPointQueueSize = 128;
static volatile uint64_t pressureDmaSamplesReady;
static volatile efitick_t pressureDmaBoundaryTime;
static volatile uint32_t pressureDmaOverruns;
static volatile uint32_t pressureTriggerQueueDrops;
static cylinder_pressure::TriggerPoint pressureTriggerPoints[TriggerPointQueueSize];
static volatile uint32_t pressureTriggerHead;
static volatile uint32_t pressureTriggerTail;
static uint64_t pressureProcessedSamples;
static cylinder_pressure::TriggerPoint previousPressureTrigger{};
static bool havePreviousPressureTrigger;
static bool havePreviousPressureBin;
static int previousPressureBin;
static cylinder_pressure::Accumulator pressureAccumulator;
static uint16_t pressureTraceStaging[cylinder_pressure::AngleBinCount];
static uint16_t completedPressureTrace[cylinder_pressure::AngleBinCount] = {};
static volatile bool completedPressureTraceReady;
static volatile bool pressureDumpInProgress;
static volatile bool pressureDiscardUntilCycleEnd;

static void pressureAdcCallback(ADCDriver* adcp) {
	UNUSED(adcp);
	pressureDmaSamplesReady += PressureDmaHalfSize;
	pressureDmaBoundaryTime = getTimeNowNt();
}

static void pressureAdcErrorCallback(ADCDriver*, adcerror_t) {
	pressureDmaOverruns++;
}

static const uint32_t pressureSmpr1 =
	ADC_SMPR1_SMP_AN10(KNOCK_SAMPLE_TIME) | ADC_SMPR1_SMP_AN11(KNOCK_SAMPLE_TIME) |
	ADC_SMPR1_SMP_AN12(KNOCK_SAMPLE_TIME) | ADC_SMPR1_SMP_AN13(KNOCK_SAMPLE_TIME) |
	ADC_SMPR1_SMP_AN14(KNOCK_SAMPLE_TIME) | ADC_SMPR1_SMP_AN15(KNOCK_SAMPLE_TIME);

static const uint32_t pressureSmpr2 =
	ADC_SMPR2_SMP_AN0(KNOCK_SAMPLE_TIME) | ADC_SMPR2_SMP_AN1(KNOCK_SAMPLE_TIME) |
	ADC_SMPR2_SMP_AN2(KNOCK_SAMPLE_TIME) | ADC_SMPR2_SMP_AN3(KNOCK_SAMPLE_TIME) |
	ADC_SMPR2_SMP_AN4(KNOCK_SAMPLE_TIME) | ADC_SMPR2_SMP_AN5(KNOCK_SAMPLE_TIME) |
	ADC_SMPR2_SMP_AN6(KNOCK_SAMPLE_TIME) | ADC_SMPR2_SMP_AN7(KNOCK_SAMPLE_TIME) |
	ADC_SMPR2_SMP_AN8(KNOCK_SAMPLE_TIME) | ADC_SMPR2_SMP_AN9(KNOCK_SAMPLE_TIME);

static const ADCConversionGroup pressureAdcGroup = {
	.circular = TRUE,
	.num_channels = 1,
	.end_cb = pressureAdcCallback,
	.error_cb = pressureAdcErrorCallback,
	.cr1 = 0,
	.cr2 = ADC_CR2_SWSTART,
	.smpr1 = pressureSmpr1,
	.smpr2 = pressureSmpr2,
	.htr = 0,
	.ltr = 0,
	.sqr1 = 0,
	.sqr2 = 0,
	.sqr3 = ADC_SQR3_SQ1_N(KNOCK_ADC_CH1),
};

static void startPressureAdc() {
	if (KNOCK_ADC.state != ADC_READY && KNOCK_ADC.state != ADC_ERROR) {
		warning(ObdCode::CUSTOM_ERR_UNEXPECTED_SHAFT_EVENT, "Pressure ADC3 already in use");
		return;
	}

	pressureDmaSamplesReady = 0;
	pressureProcessedSamples = 0;
	pressureDmaOverruns = 0;
	pressureTriggerQueueDrops = 0;
	pressureTriggerHead = 0;
	pressureTriggerTail = 0;
	efiSetPadMode("cylinder pressure", KNOCK_PIN_CH1, PAL_MODE_INPUT_ANALOG);
	adcStartConversion(&KNOCK_ADC, &pressureAdcGroup, softwareKnockSampleBuffer, PressureDmaSize);
	addConsoleAction("cylinder_pressure_dump", dumpCylinderPressureTrace);
}

static void queuePressureTrigger(efitick_t timestamp, float angle) {
	const uint32_t head = pressureTriggerHead;
	const uint32_t tail = pressureTriggerTail;
	if (head - tail >= TriggerPointQueueSize) {
		pressureTriggerQueueDrops++;
		return;
	}
	pressureTriggerPoints[head % TriggerPointQueueSize] = {
		static_cast<uint64_t>(timestamp), angle
	};
	pressureTriggerHead = head + 1;
}

static void processPressureSamples() {
	uint64_t produced;
	efitick_t boundaryTime;
	{
		chibios_rt::CriticalSectionLocker csl;
		produced = pressureDmaSamplesReady;
		boundaryTime = pressureDmaBoundaryTime;
	}
	if (produced < pressureProcessedSamples) {
		return;
	}
	if (engine->rpmCalculator.getCachedRpm() <= 0) {
		pressureProcessedSamples = produced;
		havePreviousPressureTrigger = false;
		havePreviousPressureBin = false;
		pressureAccumulator.clear();
		pressureDiscardUntilCycleEnd = false;
		pressureTriggerTail = pressureTriggerHead;
		return;
	}

	// If DMA has advanced one complete ring or more beyond the consumer, the
	// oldest unread sample is being overwritten. Discard stale data and resync.
	if (cylinder_pressure::dmaConsumerOverrun(produced, pressureProcessedSamples,
			PressureDmaSize)) {
		pressureDmaOverruns++;
		pressureProcessedSamples = produced - PressureDmaHalfSize;
		havePreviousPressureTrigger = false;
		havePreviousPressureBin = false;
		pressureAccumulator.clear();
		pressureTriggerTail = pressureTriggerHead;
	}

	const float samplePeriodNt = float(US_TO_NT_MULTIPLIER) * 1000000.0f / KNOCK_SAMPLE_RATE;
	const float cycleAngle = engine->engineState.engineCycle;
	if (cycleAngle != cylinder_pressure::AngleBinCount || samplePeriodNt <= 0) {
		return;
	}

	while (pressureTriggerTail != pressureTriggerHead) {
		const auto point = pressureTriggerPoints[pressureTriggerTail % TriggerPointQueueSize];
		if (!havePreviousPressureTrigger) {
			previousPressureTrigger = point;
			havePreviousPressureTrigger = true;
			pressureTriggerTail++;
			continue;
		}

		if (point.time <= previousPressureTrigger.time) {
			previousPressureTrigger = point;
			pressureTriggerTail++;
			continue;
		}

		const efitick_t newestSampleTime = boundaryTime;
		if (point.time > static_cast<uint64_t>(newestSampleTime)) {
			break;
		}

		while (pressureProcessedSamples < produced) {
			const uint64_t sampleIndex = pressureProcessedSamples;
			const efitick_t sampleTime = boundaryTime - static_cast<efitick_t>(
				float(produced - 1 - sampleIndex) * samplePeriodNt);
			if (static_cast<uint64_t>(sampleTime) > point.time) {
				break;
			}

			float angle;
			if (static_cast<uint64_t>(sampleTime) >= previousPressureTrigger.time &&
				cylinder_pressure::interpolateAngle(static_cast<uint64_t>(sampleTime),
					previousPressureTrigger, point, cycleAngle, angle)) {
				const int bin = static_cast<int>(angle);
				if (havePreviousPressureBin && bin < previousPressureBin) {
					if (pressureDumpInProgress || pressureDiscardUntilCycleEnd) {
						pressureAccumulator.clear();
						if (!pressureDumpInProgress) {
							pressureDiscardUntilCycleEnd = false;
						}
					} else {
						pressureAccumulator.finishCycle(pressureTraceStaging);
						chibios_rt::CriticalSectionLocker csl;
						if (!pressureDumpInProgress) {
							memcpy(completedPressureTrace, pressureTraceStaging, sizeof(completedPressureTrace));
							completedPressureTraceReady = true;
						}
					}
				}
				if (!pressureDumpInProgress && !pressureDiscardUntilCycleEnd) {
					pressureAccumulator.add(softwareKnockSampleBuffer[
						cylinder_pressure::dmaRingIndex(sampleIndex, PressureDmaSize)], angle);
				}
				previousPressureBin = bin;
				havePreviousPressureBin = true;
			}

			pressureProcessedSamples++;
		}

		if (pressureProcessedSamples == produced) {
			previousPressureTrigger = point;
			pressureTriggerTail++;
		} else {
			const efitick_t nextSampleTime = boundaryTime - static_cast<efitick_t>(
				float(produced - pressureProcessedSamples) * samplePeriodNt);
			if (static_cast<uint64_t>(nextSampleTime) > point.time) {
				previousPressureTrigger = point;
				pressureTriggerTail++;
			}
		}
	}
}

#endif // F7 continuous pressure acquisition


/**
 * this instance does not have a real physical pin - it's only used for engine sniffer
 *
 * todo: we can kind of add real physical pin just for a very narrow case of troubleshooting but only if we ever need it :)
 */
static NamedOutputPin mapAveragingPin("map");

// allow smoothing up to number of cylinders
#define MAX_MAP_BUFFER_LENGTH (MAX_CYLINDER_COUNT)
// in MAP units, not voltage!
static float averagedMapRunningBuffer[MAX_MAP_BUFFER_LENGTH];
static int mapMinBufferLength = 0;
static int averagedMapBufIdx = 0;


static void endAveraging(MapAverager* arg);

static size_t currentMapAverager = 0;

void startAveraging(mapSampler* s) {
    if (!engine->engineState.mapAveragingDuration) {
        // Zero duration means the engine wasn't spinning or something, abort
        return;
    }
	efiAssertVoid(ObdCode::CUSTOM_ERR_6649, hasLotsOfRemainingStack(), "lowstck#9");

	// TODO: set currentMapAverager based on cylinder bank
	auto& averager = getMapAvg(currentMapAverager);
	averager.start(s->cylinderNumber);

	mapAveragingPin.setHigh();
	engine->outputChannels.isMapAveraging = true;

	scheduleByAngle(&s->timer, getTimeNowNt(), engine->engineState.mapAveragingDuration,
		action_s::make<endAveraging>(&averager));
}

void MapAverager::start(uint8_t cylinderNumber) {
	chibios_rt::CriticalSectionLocker csl;

	m_counter = 0;
	m_sum = 0;
	m_isAveraging = true;
	m_cylinderNumber = cylinderNumber;
}

SensorResult MapAverager::submit(float volts) {
	auto result = m_function ? m_function->convert(volts) : unexpected;

	if (m_isAveraging && result) {
		chibios_rt::CriticalSectionLocker csl;

		m_counter++;
		m_sum += result.Value;
	}

	return result;
}

static ExpAverage expAverage;

// huh? why is this killing unit tests _linking_ only on WINDOWS?! PUBLIC_API_WEAK
float filterMapValue(float value) {
  expAverage.setSmoothingFactor(engineConfiguration->mapExpAverageAlpha);
  return expAverage.initOrAverage(value);
}

void MapAverager::stop() {
	chibios_rt::CriticalSectionLocker csl;

	m_isAveraging = false;

	if (m_counter > 0) {
		float averageMap = m_sum / m_counter;
		m_lastCounter = m_counter;

		// TODO: this should be per-sensor, not one for all MAP sensors
		averagedMapRunningBuffer[averagedMapBufIdx] = averageMap;
		// increment circular running buffer index
		averagedMapBufIdx = (averagedMapBufIdx + 1) % mapMinBufferLength;
		// find min. value (only works for pressure values, not raw voltages!)
		float minPressure = averagedMapRunningBuffer[0];
		for (int i = 1; i < mapMinBufferLength; i++) {
			if (averagedMapRunningBuffer[i] < minPressure)
				minPressure = averagedMapRunningBuffer[i];
		}

    engine->outputChannels.mapPerCylinder[m_cylinderNumber] = minPressure;
		setValidValue(filterMapValue(minPressure), getTimeNowNt());
	} else {
#if EFI_PROD_CODE
		warning(ObdCode::CUSTOM_UNEXPECTED_MAP_VALUE, "No MAP values to average");
#endif
	}
}

#if HAL_USE_ADC

/**
 * This method is invoked from ADC callback.
 * @note This method is invoked OFTEN, this method is a potential bottleneck - the implementation should be
 * as fast as possible
 */
void mapAveragingAdcCallback(float instantVoltage) {
	efiAssertVoid(ObdCode::CUSTOM_ERR_6650, hasLotsOfRemainingStack(), "lowstck#9a");

	SensorResult mapResult = getMapAvg(currentMapAverager).submit(instantVoltage);

	if (!mapResult) {
		// hopefully this warning is not too much CPU consumption for fast ADC callback
		warning(ObdCode::CUSTOM_INSTANT_MAP_DECODING, "Invalid MAP at %f", instantVoltage);
		engine->outputChannels.isMapValid = false;
	} else {
		engine->outputChannels.isMapValid = true;
	}

#if EFI_TUNER_STUDIO
	float instantMap = mapResult.value_or(0);
	engine->outputChannels.instantMAPValue = instantMap;
#endif // EFI_TUNER_STUDIO
}
#endif

static void endAveraging(MapAverager* arg) {
	arg->stop();

	engine->outputChannels.isMapAveraging = false;
	mapAveragingPin.setLow();
}

static void applyMapMinBufferLength() {
	// check range
	mapMinBufferLength = maxI(minI(engineConfiguration->mapMinBufferLength, MAX_MAP_BUFFER_LENGTH), 1);
	// reset index
	averagedMapBufIdx = 0;
	// fill with maximum values
	for (int i = 0; i < mapMinBufferLength; i++) {
		averagedMapRunningBuffer[i] = FLT_MAX;
	}
}

void MapAveragingModule::onFastCallback() {
#if EFI_PROD_CODE && EFI_CYLINDER_PRESSURE && HAL_USE_ADC && !defined(STM32H7XX)
	processPressureSamples();
#endif
	float rpm = Sensor::getOrZero(SensorType::Rpm);
    MAP_sensor_config_s * c = &engineConfiguration->map;
    angle_t start = interpolate2d(rpm, c->samplingAngleBins, c->samplingAngle);
    efiAssertVoid(ObdCode::CUSTOM_ERR_MAP_START_ASSERT, !std::isnan(start), "start");

	for (size_t i = 0; i < engineConfiguration->cylindersCount; i++) {
        float cylinderStart = start + engine->cylinders[i].getAngleOffset();
        wrapAngle(cylinderStart, "cylinderStart", ObdCode::CUSTOM_ERR_6562);
        engine->engineState.mapAveragingStart[i] = cylinderStart;
    }

    angle_t duration = interpolate2d(rpm, c->samplingWindowBins, c->samplingWindow);
    assertAngleRange(duration, "samplingDuration", ObdCode::CUSTOM_ERR_6563);

    // Clamp the duration to slightly less than one cylinder period
    float cylinderPeriod = engine->engineState.engineCycle / engineConfiguration->cylindersCount;
    engine->engineState.mapAveragingDuration = clampF(10, duration, cylinderPeriod - 10);
}

// Callback to schedule the start of map averaging for each cylinder
void MapAveragingModule::onEnginePhase(float /*rpm*/,
						efitick_t edgeTimestamp,
						float currentPhase,
						float nextPhase) {
#if EFI_PROD_CODE && EFI_CYLINDER_PRESSURE && HAL_USE_ADC && !defined(STM32H7XX)
	queuePressureTrigger(edgeTimestamp, currentPhase);
#endif
	if (!engineConfiguration->isMapAveragingEnabled) {
		return;
	}

	ScopePerf perf(PE::MapAveragingTriggerCallback);

	int samplingCount = engineConfiguration->measureMapOnlyInOneCylinder ? 1 : engineConfiguration->cylindersCount;

	for (int i = 0; i < samplingCount; i++) {
		angle_t samplingStart = engine->engineState.mapAveragingStart[i];

		if (!isPhaseInRange(samplingStart, currentPhase, nextPhase)) {
			continue;
		}

		float angleOffset = samplingStart - currentPhase;
		if (angleOffset < 0) {
			angleOffset += engine->engineState.engineCycle;
		}

		auto& s = samplers[i];

		scheduleByAngle(&s.timer, edgeTimestamp, angleOffset, action_s::make<startAveraging>(&s));
	}
}

void MapAveragingModule::onConfigurationChange(engine_configuration_s const * previousConfig) {
	if (!previousConfig || engineConfiguration->mapMinBufferLength != previousConfig->mapMinBufferLength) {
		applyMapMinBufferLength();
	}
}

void MapAveragingModule::init() {
	  for (size_t cylinderIndex = 0; cylinderIndex < MAX_CYLINDER_COUNT; cylinderIndex++) {
  		samplers[cylinderIndex].cylinderNumber = cylinderIndex;
	  }

	if (engineConfiguration->isMapAveragingEnabled) {
		efiPrintf("initMapAveraging...");
		applyMapMinBufferLength();
	} else {
		efiPrintf("Running without MapAveraging...");
	}
#if EFI_PROD_CODE && EFI_CYLINDER_PRESSURE && HAL_USE_ADC && !defined(STM32H7XX)
	startPressureAdc();
#endif
}

#else
void MapAveragingModule::onFastCallback(){}
void MapAveragingModule::onConfigurationChange(engine_configuration_s const *){}
void MapAveragingModule::init() {}
void MapAveragingModule::onEnginePhase(float, efitick_t, float, float){}
#endif /* EFI_MAP_AVERAGING */

void dumpCylinderPressureTrace() {
#if EFI_MAP_AVERAGING && defined(MODULE_MAP_AVERAGING) && EFI_PROD_CODE && EFI_CYLINDER_PRESSURE && HAL_USE_ADC && !defined(STM32H7XX)
	bool traceReady;
	uint32_t dmaOverruns;
	uint32_t triggerDrops;
	{
		chibios_rt::CriticalSectionLocker csl;
		pressureDumpInProgress = true;
		traceReady = completedPressureTraceReady;
		dmaOverruns = pressureDmaOverruns;
		triggerDrops = pressureTriggerQueueDrops;
	}
	efiPrintf("# ready=%d dma_overruns=%lu trigger_drops=%lu", traceReady ? 1 : 0,
		static_cast<unsigned long>(dmaOverruns), static_cast<unsigned long>(triggerDrops));
	efiPrintf("angle_deg,raw_adc");
	for (int i = 0; i < cylinder_pressure::AngleBinCount; i++) {
		uint16_t value;
		{
			chibios_rt::CriticalSectionLocker csl;
			value = completedPressureTrace[i];
		}
		efiPrintf("%d,%u", i, static_cast<unsigned int>(value));
	}
	{
		chibios_rt::CriticalSectionLocker csl;
		pressureDumpInProgress = false;
		pressureDiscardUntilCycleEnd = true;
	}
#else
	efiPrintf("Cylinder pressure capture is unavailable on this build");
#endif
}
