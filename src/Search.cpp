#include "Search.h"

void Search::start(Motor& motor) {
	paused_ = false;
	motor.enable(true);
	motor.setSpeedStepsPerSec(cfg_.speedStepsPerSec);
	const long pos = motor.positionSteps();
	// Reverse increases position; Forward decreases. Sweep low→high uses Reverse first.
	if (pos < cfg_.minSteps) {
		motor.setDirection(Motor::Direction::Reverse);
	} else if (pos > cfg_.maxSteps) {
		motor.setDirection(Motor::Direction::Forward);
	} else {
		motor.setDirection(Motor::Direction::Reverse);
	}
}

void Search::update(Motor& motor) {
	if (paused_) return;

	const long pos = motor.positionSteps();
	if (motor.direction() == Motor::Direction::Reverse) {
		if (pos >= cfg_.maxSteps) {
			motor.setDirection(Motor::Direction::Forward);
		}
	} else {
		if (pos <= cfg_.minSteps) {
			motor.setDirection(Motor::Direction::Reverse);
		}
	}
}

void Search::pause(Motor& motor) {
	paused_ = true;
	motor.setSpeedStepsPerSec(0);
}

void Search::resume(Motor& motor) {
	start(motor);
}
