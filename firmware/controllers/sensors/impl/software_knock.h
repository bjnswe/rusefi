/**
 * @file software_knock.h
 * @brief Interface for software-based knock detection.
 */

#pragma once

#include "pch.h"
#include <cstdint>

#include "fft/fft.h"

#define FFT_SIZE 1024

constexpr size_t SoftwareKnockSampleBufferSize = 1800;
extern adcsample_t softwareKnockSampleBuffer[SoftwareKnockSampleBufferSize];

struct SpectrogramData {
	fft::complex_type fftBuffer[FFT_SIZE];
	float window[FFT_SIZE];
};

void initSoftwareKnock();
void knockSamplingCallback(uint8_t cylinderIndex, efitick_t nowNt);
