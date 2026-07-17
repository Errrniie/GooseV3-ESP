#pragma once

#include <Arduino.h>

class Motor {
public:
	enum class Direction : uint8_t { Forward = 1, Reverse = 0 };

	Motor(uint8_t stepPin, uint8_t dirPin);

	void begin();

	void enable(bool enabled);
	bool isEnabled() const { return enabled_; }

	void setDirection(Direction dir);
	Direction direction() const { return direction_; }

	// Step rate in steps/sec. Set to 0 to stop.
	void setSpeedStepsPerSec(float stepsPerSec);
	float speedStepsPerSec() const { return speedStepsPerSec_; }

	// Call frequently from loop() to emit steps.
	void update();

	// Position tracking (in emitted step edges). Reverse steps increase position; Forward decrease.
	long positionSteps() const { return positionSteps_; }
	void setPositionSteps(long pos) { positionSteps_ = pos; }

private:
	uint8_t stepPin_;
	uint8_t dirPin_;

	bool enabled_ = false;
	Direction direction_ = Direction::Forward;
	float speedStepsPerSec_ = 0.0f;

	long positionSteps_ = 0;

	// Timing
	uint32_t stepIntervalUs_ = 0; // derived from speed
	uint32_t nextEdgeDueUs_ = 0;
	bool stepHigh_ = false;

	static constexpr uint16_t pulseHighUs_ = 5;  // STEP HIGH pulse width (µs)
	static constexpr uint16_t dirSetupUs_ = 5;   // small guard after dir change
};
