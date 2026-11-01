#pragma once

#include <stdint.h>
#include <stddef.h>

// Hardware-independent core for crank-angle pressure trace reconstruction.
// Times are in the ECU's native timer ticks; callers must provide consecutive,
// synchronized trigger points and timestamps in the same time base.
namespace cylinder_pressure {

constexpr int AngleBinCount = 720;

inline bool dmaConsumerOverrun(uint64_t produced, uint64_t consumed, size_t safeLag) {
	return produced >= consumed && produced - consumed >= safeLag;
}

inline size_t dmaRingIndex(uint64_t sequence, size_t capacity) {
	return sequence % capacity;
}

struct TriggerPoint {
	uint64_t time;
	float angle;
};

inline bool interpolateAngle(uint64_t sampleTime, const TriggerPoint& before,
		const TriggerPoint& after, float cycleAngle, float& result) {
	if (cycleAngle <= 0 || after.time <= before.time || sampleTime < before.time || sampleTime > after.time) {
		return false;
	}

	float angleDelta = after.angle - before.angle;
	while (angleDelta < 0) {
		angleDelta += cycleAngle;
	}
	while (angleDelta >= cycleAngle) {
		angleDelta -= cycleAngle;
	}

	const float fraction = static_cast<float>(sampleTime - before.time) /
		static_cast<float>(after.time - before.time);
	result = before.angle + fraction * angleDelta;
	while (result >= cycleAngle) {
		result -= cycleAngle;
	}
	return true;
}

class Accumulator {
public:
	void add(uint16_t value, float angle) {
		if (angle < 0 || angle >= AngleBinCount) {
			return;
		}

		const int bin = static_cast<int>(angle);
		sums[bin] += value;
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
	uint64_t sums[AngleBinCount] = {};
	uint32_t counts[AngleBinCount] = {};
};

} // namespace cylinder_pressure
