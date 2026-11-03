/*
 * @file test_map_averaging.cpp
 *
 * @date: may 12, 2025
 * @author FDSoftware
 */

#include "pch.h"
#include "map_averaging.h"
#include "cylinder_pressure_core.h"
#include "harley.h"

namespace {
	auto const startAveragingAction{ action_s::make<startAveraging>((mapSampler*){}) };
}

TEST(CylinderPressure, InterpolatesAcrossCycleWrapAndMissingToothGap) {
	cylinder_pressure::TriggerPoint before{1000, 710};
	cylinder_pressure::TriggerPoint after{1300, 20};
	float angle = -1;

	ASSERT_TRUE(cylinder_pressure::interpolateAngle(1150, before, after, 720, angle));
	EXPECT_NEAR(5, angle, 0.001f);

	// The larger time and angle gap models interpolation across a missing-tooth gap.
	cylinder_pressure::TriggerPoint gapEnd{1900, 80};
	ASSERT_TRUE(cylinder_pressure::interpolateAngle(1600, after, gapEnd, 720, angle));
	EXPECT_NEAR(50, angle, 0.001f);
}

TEST(CylinderPressure, InterpolatesWithChangingCrankSpeed) {
	cylinder_pressure::TriggerPoint first{100, 0};
	cylinder_pressure::TriggerPoint second{200, 90};
	cylinder_pressure::TriggerPoint third{400, 180};
	float angle = -1;

	ASSERT_TRUE(cylinder_pressure::interpolateAngle(150, first, second, 720, angle));
	EXPECT_NEAR(45, angle, 0.001f);
	ASSERT_TRUE(cylinder_pressure::interpolateAngle(300, second, third, 720, angle));
	EXPECT_NEAR(135, angle, 0.001f);
}

TEST(CylinderPressure, AccumulatesPerDegreeAndClearsEmptyBins) {
	cylinder_pressure::Accumulator accumulator;
	uint16_t result[cylinder_pressure::AngleBinCount] = {};

	accumulator.add(100, 0.1f);
	accumulator.add(200, 0.9f);
	accumulator.add(500, 719.9f);
	accumulator.finishCycle(result);

	EXPECT_EQ(150, result[0]);
	EXPECT_EQ(0, result[1]);
	EXPECT_EQ(500, result[719]);

	accumulator.add(50, 0.5f);
	accumulator.finishCycle(result);
	EXPECT_EQ(50, result[0]);
	EXPECT_EQ(0, result[719]);
}

TEST(CylinderPressure, DetectsDmaOverrunAndWrapsRingIndex) {
	EXPECT_FALSE(cylinder_pressure::dmaConsumerOverrun(4095, 0, 4096));
	EXPECT_TRUE(cylinder_pressure::dmaConsumerOverrun(4096, 0, 4096));
	EXPECT_EQ(0, cylinder_pressure::dmaRingIndex(4096, 4096));
	EXPECT_EQ(7, cylinder_pressure::dmaRingIndex(4103, 4096));
}

TEST(EngineModules, MapAveragingModule_onEnginePhase) {
    EngineTestHelper eth(engine_type_e::TEST_CRANK_ENGINE);
    engineConfiguration->isMapAveragingEnabled = true;
    engineConfiguration->measureMapOnlyInOneCylinder = true;
	engine->engineState.mapAveragingDuration = 100;
	engine->rpmCalculator.setRpmValue(200);

	engine->module<MapAveragingModule>()->onEnginePhase(200, getTimeNowNt(), 50.f, 180.f);

	// check for startMapAveraging schedule at 50° on the future (since onEnginePhase was called 50° late [start angle is 100°])
    bool averageDone = eth.assertEventExistsAtEnginePhase("startMapAveraging callback", startAveragingAction, static_cast<angle_t>(50));
    EXPECT_TRUE(averageDone);

    // move forward, we expect that the startAveraging is called and we are currently running the averaging code
	eth.moveTimeForwardMs(50);
	eth.executeActions();
	EXPECT_TRUE(engine->outputChannels.isMapAveraging);

    // move forward, here we expect the averaging is done and endAveraging was called
   	eth.moveTimeForwardMs(50);
	eth.executeActions();
    EXPECT_FALSE(engine->outputChannels.isMapAveraging);
}

TEST(EngineModules, MapAveragingModule_onFastCallback) {
    EngineTestHelper eth(engine_type_e::TEST_CRANK_ENGINE);
    engineConfiguration->isMapAveragingEnabled = true;

    MapAveragingModule mapModule;

    // trigger events at crank speed
    for (size_t i = 0; i < 9; i++) {
        eth.fireTriggerEventsWithDuration(200);
        eth.executeActions();
    }
    ASSERT_EQ(150,  Sensor::getOrZero(SensorType::Rpm));

    mapModule.onFastCallback();

    // we expect here than the map start angles correspond to the phase of the cylinder + 100 of samplingAngle
   	EXPECT_EQ(engine->engineState.mapAveragingStart[0], 100);
    EXPECT_EQ(engine->engineState.mapAveragingStart[1], 640);
    EXPECT_EQ(engine->engineState.mapAveragingStart[2], 280);
    EXPECT_EQ(engine->engineState.mapAveragingStart[3], 460);

    EXPECT_EQ(engine->engineState.mapAveragingDuration, 50);
}

TEST(EngineModules, MapAveragingModule_onFastCallbackCustomSampleWindow) {
    EngineTestHelper eth(engine_type_e::TEST_CRANK_ENGINE);
    engineConfiguration->isMapAveragingEnabled = true;
	setArrayValues(engineConfiguration->map.samplingAngle, 75);

    MapAveragingModule mapModule;

    // trigger events at crank speed
    for (size_t i = 0; i < 9; i++) {
        eth.fireTriggerEventsWithDuration(200);
        eth.executeActions();
    }
    ASSERT_EQ(150,  Sensor::getOrZero(SensorType::Rpm));

    mapModule.onFastCallback();

    // we expect here than the map start angles correspond to the phase of the cylinder + 75 of samplingAngle
    EXPECT_EQ(engine->engineState.mapAveragingStart[0], 75);  // 0 + 75
    EXPECT_EQ(engine->engineState.mapAveragingStart[1], 615); // 540 + 75
    EXPECT_EQ(engine->engineState.mapAveragingStart[2], 255); // 180 + 75
    EXPECT_EQ(engine->engineState.mapAveragingStart[3], 435); // 360 + 75

    EXPECT_EQ(engine->engineState.mapAveragingDuration, 50);
}

TEST(EngineModules, MapAveragingModule_onFastCallbackOddFire) {
    EngineTestHelper eth(engine_type_e::TEST_CRANK_ENGINE);
    engineConfiguration->isMapAveragingEnabled = true;
	setArrayValues(engineConfiguration->map.samplingAngle, 75);

	setHarley();
	engineConfiguration->timing_offset_cylinder[0] = -HARLEY_V_TWIN / 2;
	engineConfiguration->timing_offset_cylinder[1] = +HARLEY_V_TWIN / 2;

    MapAveragingModule mapModule;

    // trigger events at crank speed
    for (size_t i = 0; i < 9; i++) {
        eth.fireTriggerEventsWithDuration(200);
        eth.executeActions();
    }
    ASSERT_EQ(150,  Sensor::getOrZero(SensorType::Rpm));

    mapModule.onFastCallback();

    // 0 from getAngleOffset + 75 custom sampling start - 22.5 from timing_offset_cylinder
    EXPECT_EQ(engine->engineState.mapAveragingStart[0], 52.5);
    // 540 from getAngleOffset + 75 custom sampling start + 22.5 from timing_offset_cylinder
    EXPECT_EQ(engine->engineState.mapAveragingStart[1], 637.5);

    EXPECT_EQ(engine->engineState.mapAveragingDuration, 50);
}

TEST(EngineModules, MapAveragingModule_onEnginePhase60_2_one_cylinder) {
    EngineTestHelper eth(engine_type_e::TEST_CRANK_ENGINE);
    engineConfiguration->isMapAveragingEnabled = true;
    engineConfiguration->measureMapOnlyInOneCylinder = true;
	eth.setTriggerType(trigger_type_e::TT_TOOTHED_WHEEL_60_2);
	MapAveragingModule mapModule;
	engine->rpmCalculator.setRpmValue(200);

	engine->module<MapAveragingModule>()->onEnginePhase(200, getTimeNowNt(), 0.f, 180.f);

	// we expect offset of enginePhase (0 since we call onEnginePhase directy) + 100° of samplingAngle (default setting)
	bool averageDone = eth.assertEventExistsAtEnginePhase("startMapAveraging callback", startAveragingAction, static_cast<angle_t>(100));
    EXPECT_TRUE(averageDone);
}

TEST(EngineModules, MapAveragingModule_onEnginePhase60_2_one_cylinderCustomSampleWindow) {
    EngineTestHelper eth(engine_type_e::TEST_CRANK_ENGINE);
    engineConfiguration->isMapAveragingEnabled = true;
    engineConfiguration->measureMapOnlyInOneCylinder = true;
	eth.setTriggerType(trigger_type_e::TT_TOOTHED_WHEEL_60_2);
	MapAveragingModule mapModule;
	engine->rpmCalculator.setRpmValue(200);
	setArrayValues(engineConfiguration->map.samplingAngle, 75);

	engine->module<MapAveragingModule>()->onFastCallback();
	engine->module<MapAveragingModule>()->onEnginePhase(200, getTimeNowNt(), 0.f, 180.f);

	// we expect offset of enginePhase (0 since we call onEnginePhase directy) + 75° of samplingAngle
	bool averageDone = eth.assertEventExistsAtEnginePhase("startMapAveraging callback", startAveragingAction, static_cast<angle_t>(75));
    EXPECT_TRUE(averageDone);
}
