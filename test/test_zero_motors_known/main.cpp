// Home X/Y in lockstep, zero together, then AUTO-move to a fixed known point
// (X +339, Y +282) with no manual input. Press 'r' to reset → it re-homes and
// repeats. Purpose: shoot the same commanded point over and over to measure
// home-to-shot repeatability.

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Endstop.h"
#include "SP_System.h"

// After each home the axes are zeroed at (0,0), then driven to this fixed point.
// Values are in 16x-microstep PULSES (positive = same convention as the X100/Y20
// jog in test_zero_motors):
//   X 5424 = 339 * 16   |   Y 4512 = 282 * 16   (the old 1x aim point, scaled).
// NOTE: if you change Config::microstepping (currently 16), rescale these to match.
static constexpr long kKnownX = 5424;
static constexpr long kKnownY = 4512;

enum class AppPhase : uint8_t { Homing, AutoMove, Done };
enum class HomingPhase : uint8_t { Seek, Backoff, Reapproach, Done };

Motor motorX(Config::X::stepPin, Config::X::dirPin);
Motor motorY(Config::Y::stepPin, Config::Y::dirPin);

Endstop endstopX(Config::X::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);
Endstop endstopY(Config::Y::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);

struct AxisHoming {
	Motor& motor;
	Endstop& endstop;
	const char* name;
	bool phaseDone = false;
	bool backoffReleased = false;
	long backoffTargetPos = 0;
	// Diagnostics: where each endstop press latched, in motor steps.
	long seekTriggerPos = 0;
	long reapproachTriggerPos = 0;

	explicit AxisHoming(Motor& motorIn, Endstop& endstopIn, const char* nameIn)
		: motor(motorIn), endstop(endstopIn), name(nameIn) {}

	Motor::Direction awayDir() const {
		return (Config::Z::homeDir == Motor::Direction::Forward)
				? Motor::Direction::Reverse
				: Motor::Direction::Forward;
	}

	void stopMotor() {
		motor.setSpeedStepsPerSec(0);
	}

	void startSeek() {
		phaseDone = false;
		motor.enable(true);
		motor.setDirection(Config::Z::homeDir);
		motor.setSpeedStepsPerSec(Config::Z::seekSpeedStepsPerSec);
	}

	void startBackoff() {
		phaseDone = false;
		backoffReleased = false;
		motor.enable(true);
		motor.setDirection(awayDir());
		motor.setSpeedStepsPerSec(Config::Z::backoffSpeedStepsPerSec);
	}

	void startReapproach() {
		phaseDone = false;
		motor.enable(true);
		motor.setDirection(Config::Z::homeDir);
		motor.setSpeedStepsPerSec(Config::Z::reapproachSpeedStepsPerSec);
	}

	void begin() {
		endstop.begin();
		motor.enable(true);
		Serial.print('[');
		Serial.print(name);
		Serial.println("] ready");
	}

	void updateSeek() {
		endstop.update();
		if (phaseDone) return;

		if (endstop.isPressed()) {
			stopMotor();
			phaseDone = true;
			seekTriggerPos = motor.positionSteps();
			Serial.print('[');
			Serial.print(name);
			Serial.print("] endstop pressed (1st) at step ");
			Serial.print(seekTriggerPos);
			Serial.println(" — waiting for other axis");
		}
	}

	void updateBackoff() {
		endstop.update();
		if (phaseDone) return;

		if (!backoffReleased) {
			if (!endstop.isPressed()) {
				backoffReleased = true;
				const bool away = (motor.direction() == Motor::Direction::Forward);
				backoffTargetPos = motor.positionSteps()
						+ (away ? -Config::Z::backoffMarginSteps : Config::Z::backoffMarginSteps);
			}
			return;
		}

		const bool away = (motor.direction() == Motor::Direction::Forward);
		const bool reached = away ? (motor.positionSteps() <= backoffTargetPos)
								  : (motor.positionSteps() >= backoffTargetPos);
		if (reached) {
			stopMotor();
			phaseDone = true;
			Serial.print('[');
			Serial.print(name);
			Serial.println("] endstop released + margin — waiting for other axis");
		}
	}

	void updateReapproach() {
		endstop.update();
		if (phaseDone) return;

		if (endstop.isPressed()) {
			stopMotor();
			phaseDone = true;
			reapproachTriggerPos = motor.positionSteps();
			Serial.print('[');
			Serial.print(name);
			Serial.print("] endstop pressed (2nd) at step ");
			Serial.print(reapproachTriggerPos);
			Serial.print(" — delta from 1st = ");
			Serial.print(reapproachTriggerPos - seekTriggerPos);
			Serial.println(" steps — waiting for other axis");
		}
	}
};

// One axis auto-move to a target delta (same flipped convention as test_zero_motors:
// positive delta drives Forward, target mirrors as startPos - delta).
struct AxisMove {
	Motor& motor;
	const char* name;
	long targetPos = 0;
	bool active = false;

	explicit AxisMove(Motor& motorIn, const char* nameIn) : motor(motorIn), name(nameIn) {}

	void start(long deltaSteps) {
		if (deltaSteps == 0) return;

		const long startPos = motor.positionSteps();
		targetPos = startPos - deltaSteps;
		active = true;

		motor.enable(true);
		motor.setDirection(deltaSteps > 0 ? Motor::Direction::Forward : Motor::Direction::Reverse);
		motor.setSpeedStepsPerSec(Config::manualSpeedStepsPerSec);

		Serial.print("[Move ");
		Serial.print(name);
		Serial.print("] to known ");
		Serial.print(deltaSteps);
		Serial.println(" steps");
	}

	void update() {
		if (!active) return;

		const long pos = motor.positionSteps();
		const bool forward = (motor.direction() == Motor::Direction::Forward);
		const bool done = forward ? (pos <= targetPos) : (pos >= targetPos);
		if (!done) return;

		motor.setSpeedStepsPerSec(0);
		active = false;

		Serial.print("[Stop ");
		Serial.print(name);
		Serial.println("] at known point");
	}
};

AxisHoming homingX{motorX, endstopX, "X"};
AxisHoming homingY{motorY, endstopY, "Y"};
AxisMove moveX{motorX, "X"};
AxisMove moveY{motorY, "Y"};

HomingPhase homingPhase = HomingPhase::Seek;
AppPhase appPhase = AppPhase::Homing;

void startSeekPhase() {
	homingPhase = HomingPhase::Seek;
	Serial.println("[XY] seek started");
	homingX.startSeek();
	homingY.startSeek();
}

void startBackoffPhase() {
	homingPhase = HomingPhase::Backoff;
	Serial.println("[XY] both pressed (1st) — backoff started");
	homingX.startBackoff();
	homingY.startBackoff();
}

void startReapproachPhase() {
	homingPhase = HomingPhase::Reapproach;
	Serial.println("[XY] both released — reapproach started");
	homingX.startReapproach();
	homingY.startReapproach();
}

void enterAutoMove() {
	motorX.setPositionSteps(0);
	motorY.setPositionSteps(0);
	appPhase = AppPhase::AutoMove;
	homingPhase = HomingPhase::Done;

	Serial.println();
	Serial.println("=== Homed & zeroed — auto-moving to known point ===");
	Serial.print("Target: X +");
	Serial.print(kKnownX);
	Serial.print("  Y +");
	Serial.println(kKnownY);

	moveX.start(kKnownX);
	moveY.start(kKnownY);
}

void updateSyncedHoming() {
	switch (homingPhase) {
		case HomingPhase::Seek:
			homingX.updateSeek();
			homingY.updateSeek();
			if (homingX.phaseDone && homingY.phaseDone) startBackoffPhase();
			break;

		case HomingPhase::Backoff:
			homingX.updateBackoff();
			homingY.updateBackoff();
			if (homingX.phaseDone && homingY.phaseDone) startReapproachPhase();
			break;

		case HomingPhase::Reapproach:
			homingX.updateReapproach();
			homingY.updateReapproach();
			if (homingX.phaseDone && homingY.phaseDone) {
				Serial.println("[XY] both pressed (2nd) — homed");
				Serial.print("[Repeatability] seek->reapproach steps  X=");
				Serial.print(homingX.reapproachTriggerPos - homingX.seekTriggerPos);
				Serial.print("  Y=");
				Serial.println(homingY.reapproachTriggerPos - homingY.seekTriggerPos);
				enterAutoMove();
			}
			break;

		case HomingPhase::Done:
			break;
	}
}

void updateAutoMove() {
	moveX.update();
	moveY.update();

	if (moveX.active || moveY.active) return;

	appPhase = AppPhase::Done;
	Serial.println("=== At known point. Press 'r' to re-home and repeat. ===");
}

void setup() {
	Serial.begin(115200);

	pinMode(Config::X::enablePin, OUTPUT);
	digitalWrite(Config::X::enablePin, LOW);

	motorX.begin();
	motorY.begin();

	Serial.println();
	Serial.println("=== Zero motors KNOWN-point test (X/Y synced homing) ===");
	Serial.println("Homes, zeroes, then auto-moves to the fixed target. 'r' repeats.");

	homingX.begin();
	homingY.begin();
	startSeekPhase();
}

void loop() {
	SP::poll();
	motorX.update();
	motorY.update();

	switch (appPhase) {
		case AppPhase::Homing:
			updateSyncedHoming();
			break;
		case AppPhase::AutoMove:
			updateAutoMove();
			break;
		case AppPhase::Done:
			break;
	}
}
