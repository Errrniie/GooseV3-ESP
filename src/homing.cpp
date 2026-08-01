#include "Homing.h"

Homing::Homing(Motor& motor, Endstop& endstop) : Homing(motor, endstop, Config{}) {}

Homing::Homing(Motor& motor, Endstop& endstop, const Config& cfg)
		: motor_(motor), endstop_(endstop), cfg_(cfg) {}

void Homing::start() {
	errorReason_ = nullptr;
	enter_(State::SeekToEndstop);
}

void Homing::abort(const char* reason) {
	fail_(reason ? reason : "aborted");
}

const char* Homing::stateName() const {
	switch (state_) {
		case State::Idle: return "Idle";
		case State::SeekToEndstop: return "SeekToEndstop";
		case State::Backoff: return "Backoff";
		case State::Reapproach: return "Reapproach";
		case State::SetZero: return "SetZero";
		case State::MoveToFinal: return "MoveToFinal";
		case State::Done: return "Done";
		case State::Error: return "Error";
	}
	return "?";
}

void Homing::update() {
	// Make sure the motor is always being serviced.
	motor_.update();

	if (state_ == State::Idle || state_ == State::Done || state_ == State::Error) return;

	const uint32_t nowMs = millis();
	const bool pressed = endstop_.isPressed();
	const long travel = phaseTravel_();

	switch (state_) {
		case State::SeekToEndstop: {
			if ((nowMs - phaseStartMs_) > cfg_.seekTimeoutMs) return fail_("seek timeout");
			if (travel > cfg_.seekStepsMax) return fail_("seek steps max exceeded");
			if (pressed) return enter_(State::Backoff);
			return;
		}

		case State::Backoff: {
			if ((nowMs - phaseStartMs_) > cfg_.backoffTimeoutMs) return fail_("backoff timeout");
			if (travel > cfg_.backoffStepsMax) return fail_("backoff steps max exceeded");

			// Phase 1: keep moving away until the switch reports released, then latch a
			// target a fixed margin further in the away direction.
			if (!backoffReleased_) {
				if (!pressed) {
					backoffReleased_ = true;
					const bool away = (motor_.direction() == Motor::Direction::Forward);
					backoffTargetPos_ = motor_.positionSteps()
							+ (away ? -cfg_.backoffMarginSteps : cfg_.backoffMarginSteps);
				}
				return;
			}

			// Phase 2: keep going until the margin is covered (>= handles overshoot, no hang).
			const bool away = (motor_.direction() == Motor::Direction::Forward);
			const bool reached = away ? (motor_.positionSteps() <= backoffTargetPos_)
									  : (motor_.positionSteps() >= backoffTargetPos_);
			if (reached) return enter_(State::Reapproach);
			return;
		}

		case State::Reapproach: {
			if ((nowMs - phaseStartMs_) > cfg_.reapproachTimeoutMs) return fail_("reapproach timeout");
			if (travel > cfg_.reapproachStepsMax) return fail_("reapproach steps max exceeded");
			if (pressed) return enter_(State::SetZero);
			return;
		}

		case State::SetZero: {
			homeZeroPos_ = motor_.positionSteps();
			motor_.setPositionSteps(0);
			enter_(State::MoveToFinal);
			return;
		}

		case State::MoveToFinal: {
			if ((nowMs - phaseStartMs_) > cfg_.finalMoveTimeoutMs) return fail_("final move timeout");
			const long pos = motor_.positionSteps();
			const long tgt = cfg_.finalPositionSteps;
			const bool forward = (motor_.direction() == Motor::Direction::Forward);
			// Forward decreases position; Reverse increases.
			const bool done = forward ? (pos <= tgt) : (pos >= tgt);
			if (done) {
				motor_.setSpeedStepsPerSec(0);
				state_ = State::Done;
			}
			return;
		}

		default:
			return;
	}
}

void Homing::enter_(State s) {
	state_ = s;
	phaseStartPos_ = motor_.positionSteps();
	phaseStartMs_ = millis();

	switch (state_) {
		case State::SeekToEndstop:
			motor_.enable(true);
			motor_.setDirection(cfg_.homeDir);
			motor_.setSpeedStepsPerSec(cfg_.seekSpeedStepsPerSec);
			return;

		case State::Backoff:
			backoffReleased_ = false;
			motor_.enable(true);
			motor_.setDirection(awayDir_());
			motor_.setSpeedStepsPerSec(cfg_.backoffSpeedStepsPerSec);
			return;

		case State::Reapproach:
			motor_.enable(true);
			motor_.setDirection(cfg_.homeDir);
			motor_.setSpeedStepsPerSec(cfg_.reapproachSpeedStepsPerSec);
			return;

		case State::SetZero:
			motor_.setSpeedStepsPerSec(0);
			return;

		case State::MoveToFinal: {
			const long pos = motor_.positionSteps();
			const long tgt = cfg_.finalPositionSteps;
			if (pos == tgt) {
				motor_.setSpeedStepsPerSec(0);
				state_ = State::Done;
				return;
			}
			motor_.enable(true);
			if (pos < tgt) {
				motor_.setDirection(Motor::Direction::Reverse);
			} else {
				motor_.setDirection(Motor::Direction::Forward);
			}
			motor_.setSpeedStepsPerSec(cfg_.travelSpeedStepsPerSec);
			return;
		}

		default:
			return;
	}
}

void Homing::fail_(const char* reason) {
	errorReason_ = reason;
	motor_.setSpeedStepsPerSec(0);
	state_ = State::Error;
}

long Homing::phaseTravel_() const {
	const long nowPos = motor_.positionSteps();
	const long delta = nowPos - phaseStartPos_;
	return (delta >= 0) ? delta : -delta;
}

Motor::Direction Homing::awayDir_() const {
	return (cfg_.homeDir == Motor::Direction::Forward) ? Motor::Direction::Reverse : Motor::Direction::Forward;
}