#pragma once

#include <Arduino.h>
#include "Motor.h"
#include "Endstop.h"

// Homing FSM: seek → backoff → reapproach → set zero → move to finalPositionSteps → done.
class Homing {
public:
	struct Config {
		Motor::Direction homeDir = Motor::Direction::Reverse;

		float seekSpeedStepsPerSec = 800.0f;
		float backoffSpeedStepsPerSec = 300.0f;
		float reapproachSpeedStepsPerSec = 200.0f;
		float travelSpeedStepsPerSec = 1000.0f;

		long seekStepsMax = 40000;
		uint32_t seekTimeoutMs = 30000;

		long backoffStepsMax = 400;
		uint32_t backoffTimeoutMs = 15000;

		long reapproachStepsMax = 6000;
		uint32_t reapproachTimeoutMs = 15000;

		long finalPositionSteps = 500;
		uint32_t finalMoveTimeoutMs = 60000;
	};

	enum class State : uint8_t {
		Idle,
		SeekToEndstop,
		Backoff,
		Reapproach,
		SetZero,
		MoveToFinal,
		Done,
		Error
	};

	Homing(Motor& motor, Endstop& endstop);
	Homing(Motor& motor, Endstop& endstop, const Config& cfg);

	void start();
	void abort(const char* reason = "aborted");
	void update();

	bool isHomed() const { return state_ == State::Done; }
	bool isDone() const { return state_ == State::Done || state_ == State::Error; }
	bool hasError() const { return state_ == State::Error; }
	State state() const { return state_; }
	const char* stateName() const;
	const char* errorReason() const { return errorReason_; }

private:
	Motor& motor_;
	Endstop& endstop_;
	Config cfg_;

	State state_ = State::Idle;
	const char* errorReason_ = nullptr;

	long phaseStartPos_ = 0;
	uint32_t phaseStartMs_ = 0;
	long homeZeroPos_ = 0;

	void enter_(State s);
	void fail_(const char* reason);
	long phaseTravel_() const;
	Motor::Direction awayDir_() const;
};
