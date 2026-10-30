/**
 * @file	logic_analyzer.h
 *
 * @date Jan 7, 2013
 * @author Andrey Belomutskiy, (c) 2012-2020
 */

#pragma once

#include "global.h"

#if !defined(EFI_LOGIC_ANALYZER)
    #warning "EFI_LOGIC_ANALYZER is not defined!"
#elif EFI_LOGIC_ANALYZER == 0
    #warning "EFI_LOGIC_ANALYZER is 0!"
#else

#include "engine_sniffer.h"

void initWaveAnalyzer();
void startLogicAnalyzerPins();
void stopLogicAnalyzerPins();
void printWave(Logging *logging);
void showWaveInfo(void);
void reportLogicAnalyzerToTS(void); 

void waTriggerEventListener(trigger_event_e ckpSignalType, uint32_t index, efitick_t edgeTimestamp);

#endif /* EFI_LOGIC_ANALYZER */

