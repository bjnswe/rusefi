#pragma once

#include <stdint.h>
#include <stddef.h>

// Hardware-independent core for crank-angle pressure trace reconstruction.
// Times are lower-32-bit ECU timer values. Trigger intervals must be shorter
// than half the counter range so signed differences remain wrap-safe.
namespace cylinder_pressure {

constexpr int AngleBinCount = 720;

inline uint32_t dmaSamplesPending(uint32_t produced, uint32_t consumed) {
	// Unsigned subtraction intentionally handles the sequence counter wrapping.
	return produced - consumed;
}

inline bool dmaConsumerOverrun(uint32_t produced, uint32_t consumed, size_t safeLag) {
	return dmaSamplesPending(produced, consumed) >= safeLag;
}

inline size_t dmaRingIndex(uint32_t sequence, size_t capacity) {
	return sequence % capacity;
}

inline size_t dmaRingIndexAtBoundary(uint32_t sequence, uint32_t produced,
		size_t boundaryIndex, size_t capacity) {
	const uint32_t samplesBehindBoundary = produced - sequence;
	return (boundaryIndex + capacity - samplesBehindBoundary % capacity) % capacity;
}

struct TriggerPoint {
	uint32_t time;
	float angle;
};

inline bool interpolateAngle(uint32_t sampleTime, const TriggerPoint& before,
		const TriggerPoint& after, float cycleAngle, float& result) {
	// Trigger/sample times are relative 32-bit timer values. Signed differences
	// are valid as long as the interval is shorter than half the counter range.
	const int32_t triggerInterval = static_cast<int32_t>(after.time - before.time);
	const int32_t sampleOffset = static_cast<int32_t>(sampleTime - before.time);
	if (cycleAngle <= 0 || triggerInterval <= 0 || sampleOffset < 0 || sampleOffset > triggerInterval) {
		return false;
	}

	float angleDelta = after.angle - before.angle;
	while (angleDelta < 0) {
		angleDelta += cycleAngle;
	}
	while (angleDelta >= cycleAngle) {
		angleDelta -= cycleAngle;
	}

	const float fraction = static_cast<float>(sampleOffset) / static_cast<float>(triggerInterval);
	result = before.angle + fraction * angleDelta;
	while (result >= cycleAngle) {
		result -= cycleAngle;
	}
	return true;
}

class Accumulator {
public:
	static constexpr uint8_t MaxCount = 128;
	static constexpr uint32_t MaxSum = 65535;

	void add(uint16_t value, float angle) {
		if (angle < 0 || angle >= AngleBinCount) {
			return;
		}

		const int bin = static_cast<int>(angle);
    uint32_t newSum = (uint32_t)sums[bin] + value;
    
		if ((counts[bin] >= MaxCount) ||
        (newSum > MaxSum)         ) {
			return;
		}

		sums[bin] = newSum;
		counts[bin]++;
	}

	void finishCycle(uint16_t* averages) {
		for (int i = 0; i < AngleBinCount; i++) {
			averages[i] = counts[i] ? static_cast<uint16_t>(sums[i] / counts[i]) : 0;
			sums[i] = 0;
			counts[i] = 0;
		}
	}

	void clear() {
		for (int i = 0; i < AngleBinCount; i++) {
			sums[i] = 0;
			counts[i] = 0;
		}
	}

private:
	uint16_t sums[AngleBinCount] = {};
	uint8_t counts[AngleBinCount] = {};
};

} // namespace cylinder_pressure
