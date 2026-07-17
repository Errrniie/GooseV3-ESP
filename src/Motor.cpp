#include "Motor.h"

Motor::Motor(uint8_t stepPin, uint8_t dirPin) : stepPin_(stepPin), dirPin_(dirPin) {}

void Motor::begin() {
	pinMode(stepPin_, OUTPUT);
	pinMode(dirPin_, OUTPUT);
	digitalWrite(stepPin_, LOW);
	// Initialize DIR to match the current logical direction_.
	digitalWrite(dirPin_, (direction_ == Direction::Forward) ? HIGH : LOW);
	enabled_ = true;
	stepHigh_ = false;
	nextEdgeDueUs_ = micros();
}

void Motor::enable(bool enabled) {
	enabled_ = enabled;
	if (!enabled_) {
		stepHigh_ = false;
		digitalWrite(stepPin_, LOW);
	}
}

void Motor::setDirection(Direction dir) {
	if (direction_ == dir) return;
	direction_ = dir;
	digitalWrite(dirPin_, (dir == Direction::Forward) ? HIGH : LOW);

	// Guard time before the next step edge.
	const uint32_t now = micros();
	const uint32_t guardUntil = now + dirSetupUs_;
	if ((int32_t)(guardUntil - nextEdgeDueUs_) > 0) {
		nextEdgeDueUs_ = guardUntil;
	}
}

void Motor::setSpeedStepsPerSec(float stepsPerSec) {
	if (stepsPerSec < 0.0f) stepsPerSec = -stepsPerSec;
	speedStepsPerSec_ = stepsPerSec;

	if (speedStepsPerSec_ <= 0.0f) {
		stepIntervalUs_ = 0;
		return;
	}

	const float interval = 1000000.0f / speedStepsPerSec_;
	stepIntervalUs_ = (interval < 1.0f) ? 1u : static_cast<uint32_t>(interval);
}

void Motor::update() {
	if (!enabled_) return;
	if (stepIntervalUs_ == 0) return;

	const uint32_t now = micros();
	if ((int32_t)(now - nextEdgeDueUs_) < 0) return;

	if (!stepHigh_) {
		// Idle LOW -> pulse ON (HIGH).
		digitalWrite(stepPin_, HIGH);
		stepHigh_ = true;
		nextEdgeDueUs_ = now + pulseHighUs_;
		return;
	}

	// Pulse OFF (LOW); one step completed.
	digitalWrite(stepPin_, LOW);
	stepHigh_ = false;

	// Reverse direction counts as positive step movement; Forward counts negative.
	positionSteps_ += (direction_ == Direction::Reverse) ? 1 : -1;
	nextEdgeDueUs_ = now + (stepIntervalUs_ - pulseHighUs_);
}

