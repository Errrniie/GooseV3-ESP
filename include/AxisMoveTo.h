#pragma once

#include <Arduino.h>
#include "Motor.h"

// Non-blocking absolute move to a coordinate, shared by PatternShot and by any
// one-off "go to a point" step (e.g. moving to the neutral/home position).
//
// Coordinates are absolute step-pulses from home (0,0), jog convention: a
// positive coordinate C drives Forward from home and is reached at motor
// position -C. Call update() every loop(); the caller still calls motor.update()
// to emit the pulses.

class AxisMoveTo {
public:
	explicit AxisMoveTo(Motor& motor) : motor_(motor) {}

	void startTo(long coord, float speed) {
		targetPos_ = -coord;
		const long pos = motor_.positionSteps();
		if (pos == targetPos_) {            // already there
			active_ = false;
			motor_.setSpeedStepsPerSec(0);
			return;
		}
		active_ = true;
		motor_.enable(true);
		// Forward decreases position, Reverse increases it.
		motor_.setDirection(pos > targetPos_ ? Motor::Direction::Forward
											 : Motor::Direction::Reverse);
		motor_.setSpeedStepsPerSec(speed);
	}

	void update() {
		if (!active_) return;
		const long pos = motor_.positionSteps();
		const bool forward = (motor_.direction() == Motor::Direction::Forward);
		const bool done = forward ? (pos <= targetPos_) : (pos >= targetPos_);
		if (!done) return;
		motor_.setSpeedStepsPerSec(0);
		active_ = false;
	}

	void cancel() { active_ = false; }
	bool busy() const { return active_; }

private:
	Motor& motor_;
	long targetPos_ = 0;
	bool active_ = false;
};

// Move both axes to an absolute (x, y) point together; busy() until both arrive.
class MoveXY {
public:
	MoveXY(Motor& motorX, Motor& motorY) : x_(motorX), y_(motorY) {}

	void startTo(long xCoord, long yCoord, float speed) {
		x_.startTo(xCoord, speed);
		y_.startTo(yCoord, speed);
	}

	void update() {
		x_.update();
		y_.update();
	}

	void cancel() {
		x_.cancel();
		y_.cancel();
	}

	bool busy() const { return x_.busy() || y_.busy(); }

private:
	AxisMoveTo x_;
	AxisMoveTo y_;
};
