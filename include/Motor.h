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

	static constexpr uint16_t pulseHighUs_ = 5;  // STEP HIGH pulse width (µs); > driver's 1.5µs filter
	// DIR-before-STEP setup. Closed-loop drivers latch direction ~5µs before the
	// first pulse; sitting at exactly 5µs let post-reversal pulses count the wrong
	// way (probabilistic whole-step home error, worse on the marginal axis). 60µs
	// is free at homing speeds (>=25ms/step) and puts us well clear of the window.
	static constexpr uint16_t dirSetupUs_ = 60;
};
