#pragma once

#include <Arduino.h>
#include "Motor.h"
#include "Config.h"
#include "AxisMoveTo.h"

// PatternShot: drive the X and Y motors around a fixed set of four corner
// coordinates (a square "pattern shot"), rapidly, repeating the whole loop a
// configurable number of times.
//
// Coordinates are ABSOLUTE step-pulses measured from home (0,0), in the same
// "jog convention" used elsewhere (positive coordinate = Forward direction from
// home). They are raw pulses at the CURRENT microstepping — e.g. the calibrated
// point is X 5424 / Y 4512 at 16x. Feed whatever pulse counts land your corners.
//
// Non-blocking: call update() every loop() iteration, and also call
// motorX.update() / motorY.update() (the step engine) as usual. PatternShot only
// sets direction/speed and watches for arrival — it does not pulse the motors.
//
// Usage:
//   PatternShot pattern(motorX, motorY);
//   pattern.configure(corners, 30);   // 4 corners, repeat the square 30 times
//   pattern.begin();                  // start from the current position
//   ... in loop(): motorX.update(); motorY.update(); pattern.update();

struct PatternCorner {
	long x;
	long y;
};

class PatternShot {
public:
	static constexpr uint8_t kNumCorners = 4;

	PatternShot(Motor& motorX, Motor& motorY)
		: motorX_(motorX), motorY_(motorY), axisX_(motorX), axisY_(motorY) {}

	// corners: the four square targets (absolute pulses from home).
	// repeats: how many times to run the full 4-corner loop.
	// speed:   rapid traverse speed in steps/sec (default = scaled travel speed).
	void configure(const PatternCorner corners[kNumCorners], uint16_t repeats,
			float speedStepsPerSec = Config::Z::travelSpeedStepsPerSec) {
		for (uint8_t i = 0; i < kNumCorners; ++i) corners_[i] = corners[i];
		repeatsTotal_ = repeats;
		speed_ = speedStepsPerSec;
		state_ = State::Idle;
	}

	// Start the pattern from wherever the motors currently are.
	void begin() {
		if (repeatsTotal_ == 0) {
			state_ = State::Done;
			return;
		}
		repeatsDone_ = 0;
		state_ = State::Running;
		startCorner_(0);
	}

	// Halt motion and leave the pattern idle.
	void stop() {
		motorX_.setSpeedStepsPerSec(0);
		motorY_.setSpeedStepsPerSec(0);
		axisX_.cancel();
		axisY_.cancel();
		state_ = State::Idle;
	}

	// Call every loop() iteration.
	void update() {
		axisX_.update();
		axisY_.update();

		if (state_ != State::Running) return;
		if (axisX_.busy() || axisY_.busy()) return;

		// Both axes have arrived at corners_[cornerIndex_]; advance.
		uint8_t next = cornerIndex_ + 1;
		if (next >= kNumCorners) {
			next = 0;
			++repeatsDone_;
			if (repeatsDone_ >= repeatsTotal_) {
				state_ = State::Done;
				return;
			}
		}
		startCorner_(next);
	}

	bool isRunning() const { return state_ == State::Running; }
	bool isDone() const { return state_ == State::Done; }
	uint16_t repeatsDone() const { return repeatsDone_; }
	uint16_t repeatsTotal() const { return repeatsTotal_; }
	uint8_t cornerIndex() const { return cornerIndex_; }

private:
	enum class State : uint8_t { Idle, Running, Done };

	void startCorner_(uint8_t idx) {
		cornerIndex_ = idx;
		axisX_.startTo(corners_[idx].x, speed_);
		axisY_.startTo(corners_[idx].y, speed_);
	}

	Motor& motorX_;
	Motor& motorY_;
	AxisMoveTo axisX_;
	AxisMoveTo axisY_;

	PatternCorner corners_[kNumCorners] = {};
	uint16_t repeatsTotal_ = 0;
	uint16_t repeatsDone_ = 0;
	uint8_t cornerIndex_ = 0;
	float speed_ = 0.0f;
	State state_ = State::Idle;
};
