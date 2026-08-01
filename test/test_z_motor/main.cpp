// Z motor test: home (double-click, seek -> backoff -> reapproach) exactly like
// X/Y, zero at home, then TYPE a signed step count to jog Z either direction.
//   200   -> move 200 steps one way
//  -200   -> move 200 steps the other way

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Endstop.h"
#include "SP_System.h"

Motor motorZ(Config::Z::stepPin, Config::Z::dirPin);
Endstop endstopZ(Config::Z::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);

// ---------------- Single-axis homing (seek -> backoff -> reapproach) ----------
enum class HomingPhase : uint8_t { Seek, Backoff, Reapproach, Done };

struct AxisHoming {
	Motor& motor;
	Endstop& endstop;
	bool phaseDone = false;
	bool backoffReleased = false;
	long backoffTargetPos = 0;

	AxisHoming(Motor& m, Endstop& e) : motor(m), endstop(e) {}

	Motor::Direction awayDir() const {
		return (Config::Z::homeDir == Motor::Direction::Forward)
				? Motor::Direction::Reverse : Motor::Direction::Forward;
	}
	void stopMotor() { motor.setSpeedStepsPerSec(0); }

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
	void begin() { endstop.begin(); motor.enable(true); }

	void updateSeek() {
		endstop.update();
		if (phaseDone) return;
		if (endstop.isPressed()) { stopMotor(); phaseDone = true; }   // 1st click
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
		if (reached) { stopMotor(); phaseDone = true; }
	}
	void updateReapproach() {
		endstop.update();
		if (phaseDone) return;
		if (endstop.isPressed()) { stopMotor(); phaseDone = true; }   // 2nd click
	}
};

AxisHoming homingZ{motorZ, endstopZ};
HomingPhase homingPhase = HomingPhase::Seek;

enum class Phase : uint8_t { Homing, Ready };
Phase phase = Phase::Homing;

String inputBuffer;

// ---- Relative jog ----
long jogTargetPos = 0;
bool jogging = false;

void startJog(long deltaSteps) {
	if (deltaSteps == 0) {
		Serial.println("0 steps does nothing.");
		return;
	}
	const long start = motorZ.positionSteps();
	jogTargetPos = start + deltaSteps;   // motor position space: Reverse = +1, Forward = -1
	jogging = true;

	motorZ.enable(true);
	motorZ.setDirection(deltaSteps > 0 ? Motor::Direction::Reverse : Motor::Direction::Forward);
	motorZ.setSpeedStepsPerSec(Config::manualSpeedStepsPerSec);

	Serial.print("[Move Z] ");
	Serial.print(deltaSteps);
	Serial.print(" steps: ");
	Serial.print(start);
	Serial.print(" -> ");
	Serial.println(jogTargetPos);
}

void updateJog() {
	if (!jogging) return;
	const long pos = motorZ.positionSteps();
	const bool reverse = (motorZ.direction() == Motor::Direction::Reverse);
	const bool done = reverse ? (pos >= jogTargetPos) : (pos <= jogTargetPos);
	if (!done) return;

	motorZ.setSpeedStepsPerSec(0);
	jogging = false;
	Serial.print("[Z at] ");
	Serial.println(motorZ.positionSteps());
	Serial.println("Enter steps to move Z (e.g. 200 or -200):");
}

void updateHoming() {
	switch (homingPhase) {
		case HomingPhase::Seek:
			homingZ.updateSeek();
			if (homingZ.phaseDone) { homingPhase = HomingPhase::Backoff; homingZ.startBackoff(); }
			break;
		case HomingPhase::Backoff:
			homingZ.updateBackoff();
			if (homingZ.phaseDone) { homingPhase = HomingPhase::Reapproach; homingZ.startReapproach(); }
			break;
		case HomingPhase::Reapproach:
			homingZ.updateReapproach();
			if (homingZ.phaseDone) homingPhase = HomingPhase::Done;
			break;
		case HomingPhase::Done:
			break;
	}
}

void handleSerial() {
	while (Serial.available() > 0) {
		const char c = static_cast<char>(Serial.read());
		if (c == '\r') continue;
		if (c == '\n') {
			inputBuffer.trim();
			if (inputBuffer.length() > 0) {
				if (jogging) {
					Serial.println("Busy moving. Wait for it to finish.");
				} else {
					char* endPtr = nullptr;
					const long steps = strtol(inputBuffer.c_str(), &endPtr, 10);
					if (endPtr != nullptr && *endPtr == '\0') {
						startJog(steps);
					} else {
						Serial.println("Enter a whole number of steps, e.g. 200 or -200.");
					}
				}
			}
			inputBuffer = "";
			continue;
		}
		inputBuffer += c;
	}
}

void setup() {
	Serial.begin(115200);

	pinMode(Config::Z::enablePin, OUTPUT);
	// Clear any latched driver alarm: disable, pause, re-enable.
	digitalWrite(Config::Z::enablePin, HIGH);   // disable
	delay(600);
	digitalWrite(Config::Z::enablePin, LOW);    // enable
	delay(200);

	motorZ.begin();

	Serial.println();
	Serial.println("=== Z motor test (home -> jog) ===");

	homingZ.begin();
	homingZ.startSeek();
	Serial.println("Homing Z (double-click)...");
}

void loop() {
	SP::poll();
	motorZ.update();

	switch (phase) {
		case Phase::Homing:
			updateHoming();
			if (homingPhase == HomingPhase::Done) {
				motorZ.setPositionSteps(0);   // zero at endstop home
				Serial.println("Homed & zeroed at 0.");
				Serial.println("Enter steps to move Z (e.g. 200 or -200):");
				phase = Phase::Ready;
			}
			break;

		case Phase::Ready:
			handleSerial();
			updateJog();
			break;
	}
}
