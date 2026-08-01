// Aim-angle test:  home (double-click) -> move to the neutral point -> ready.
// Then, given a ground distance in INCHES typed in the terminal, use the laser
// height + the BNO055 tilt to compute the aim angle. The angle -> X-motor move
// is intentionally NOT wired yet (defined later).

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Endstop.h"
#include "AxisMoveTo.h"
#include "SP_System.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// ---- Geometry / sensor ----
static constexpr float kLaserHeightInches = 83.0f;  // 6 ft 11 in above the ground
static constexpr float kGravity = 9.8f;             // normalize accel to g
static constexpr uint8_t kSdaPin = 16;
static constexpr uint8_t kSclPin = 17;

Motor motorX(Config::X::stepPin, Config::X::dirPin);
Motor motorY(Config::Y::stepPin, Config::Y::dirPin);
Endstop endstopX(Config::X::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);
Endstop endstopY(Config::Y::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
bool bnoReady = false;

MoveXY toNeutral(motorX, motorY);

// X tilts to the aim angle. Neutral X (Config::Neutral::x) is 0 degrees.
AxisMoveTo aimX(motorX);
bool aiming = false;
// Which way is "down" for X. -1: positive distance tilts the beam DOWN (verified
// on hardware — +1 pointed up).
static constexpr int kAimDownSign = -1;

// ---------------- Synced X/Y homing (seek -> backoff -> reapproach) ----------
enum class HomingPhase : uint8_t { Seek, Backoff, Reapproach, Done };

struct AxisHoming {
	Motor& motor;
	Endstop& endstop;
	const char* name;
	bool phaseDone = false;
	bool backoffReleased = false;
	long backoffTargetPos = 0;
	// Diagnostics: microstep counts at the 1st and 2nd endstop clicks.
	long seekTriggerPos = 0;
	long reapproachTriggerPos = 0;

	AxisHoming(Motor& m, Endstop& e, const char* n) : motor(m), endstop(e), name(n) {}

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
		if (endstop.isPressed()) { stopMotor(); phaseDone = true; seekTriggerPos = motor.positionSteps(); }   // 1st click
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
		if (endstop.isPressed()) { stopMotor(); phaseDone = true; reapproachTriggerPos = motor.positionSteps(); }   // 2nd click
	}
};

AxisHoming homingX{motorX, endstopX, "X"};
AxisHoming homingY{motorY, endstopY, "Y"};
HomingPhase homingPhase = HomingPhase::Seek;

enum class Phase : uint8_t { Homing, ToNeutral, Ready };
Phase phase = Phase::Homing;

// Interactive shoot -> measure -> error loop (inside the Ready phase).
enum class AimStep : uint8_t { AskDistance, Moving, AskMeasured, AskAnother, Stopped };
AimStep aimStep = AimStep::AskDistance;
float commandedDistance = 0.0f;

String inputBuffer;

void updateSyncedHoming() {
	switch (homingPhase) {
		case HomingPhase::Seek:
			homingX.updateSeek();
			homingY.updateSeek();
			if (homingX.phaseDone && homingY.phaseDone) {
				homingPhase = HomingPhase::Backoff;
				homingX.startBackoff();
				homingY.startBackoff();
			}
			break;
		case HomingPhase::Backoff:
			homingX.updateBackoff();
			homingY.updateBackoff();
			if (homingX.phaseDone && homingY.phaseDone) {
				homingPhase = HomingPhase::Reapproach;
				homingX.startReapproach();
				homingY.startReapproach();
			}
			break;
		case HomingPhase::Reapproach:
			homingX.updateReapproach();
			homingY.updateReapproach();
			if (homingX.phaseDone && homingY.phaseDone) homingPhase = HomingPhase::Done;
			break;
		case HomingPhase::Done:
			break;
	}
}

// Current tilt off-from-parallel, in degrees (from normalized BNO055 Z).
float readTiltDeg() {
	if (!bnoReady) return 0.0f;
	imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	const float nz = constrain(a.z() / kGravity, -1.0f, 1.0f);
	return asinf(nz) * 180.0f / PI;
}

void reportAndAim(float distanceInches) {
	const float angleFromVertical = atanf(distanceInches / kLaserHeightInches) * 180.0f / PI;
	// Depression below horizontal to hit distance D: shrinks as D grows (farther
	// target = flatter beam). This is the angle to aim by.
	const float angleBelowHoriz = atanf(kLaserHeightInches / distanceInches) * 180.0f / PI;
	const float tilt = readTiltDeg();

	// Tilt compensation (in depression terms): tilting back (Z positive) raises the
	// beam, so we need MORE depression to bring it down -> add the tilt. Tilting
	// forward (Z negative) subtracts it back.
	const float effectiveAngle = angleBelowHoriz + tilt;

	// Convert angle -> steps via Config (0.1125 deg/step at 16x). Absolute move
	// from neutral (treated as 0 deg / horizontal) so each aim is fresh.
	const long steps = Config::stepsForDegrees(effectiveAngle);
	const long targetX = Config::Neutral::x + kAimDownSign * steps;

	Serial.println();
	Serial.print("Distance D = ");
	Serial.print(distanceInches, 1);
	Serial.print(" in   height H = ");
	Serial.print(kLaserHeightInches, 1);
	Serial.println(" in");
	Serial.print("  angle from vertical (atan D/H) = ");
	Serial.print(angleFromVertical, 2);
	Serial.println(" deg");
	Serial.print("  angle below horizontal         = ");
	Serial.print(angleBelowHoriz, 2);
	Serial.println(" deg");
	Serial.print("  current tilt (BNO055 Z)        = ");
	Serial.print(tilt, 2);
	Serial.println(" deg");
	Serial.print("  effective depression (+tilt)   = ");
	Serial.print(effectiveAngle, 2);
	Serial.println(" deg");
	Serial.print("  X move: ");
	Serial.print(steps);
	Serial.print(" steps (0.1125 deg/step) -> X coord ");
	Serial.println(targetX);

	aimX.startTo(targetX, Config::manualSpeedStepsPerSec);
	aiming = true;
}

void handleLine(const String& line) {
	switch (aimStep) {
		case AimStep::AskDistance: {
			commandedDistance = line.toFloat();   // parses leading number, ignores "in"
			if (commandedDistance <= 0.0f) {
				Serial.println("Enter a positive distance in inches:");
				return;
			}
			reportAndAim(commandedDistance);       // prints info + starts the X move
			aimStep = AimStep::Moving;
			break;
		}

		case AimStep::Moving:
			// Motor is moving; ignore input until it reports at target.
			break;

		case AimStep::AskMeasured: {
			const float measured = line.toFloat();
			if (measured <= 0.0f) {
				Serial.println("Enter the measured distance in inches:");
				return;
			}
			const float distErr = measured - commandedDistance;
			const float pctErr = distErr / commandedDistance * 100.0f;
			Serial.println();
			Serial.print("  commanded distance = ");
			Serial.print(commandedDistance, 1);
			Serial.println(" in");
			Serial.print("  actual (measured)  = ");
			Serial.print(measured, 1);
			Serial.println(" in");
			Serial.print("  distance error     = ");
			Serial.print(distErr, 1);
			Serial.println(" in");
			Serial.print("  percent error      = ");
			Serial.print(pctErr, 1);
			Serial.println(" %");
			Serial.println("Another shot? (y/n)");
			aimStep = AimStep::AskAnother;
			break;
		}

		case AimStep::AskAnother: {
			const char c0 = line.length() ? line.charAt(0) : 0;
			if (c0 == 'y' || c0 == 'Y') {
				Serial.println();
				Serial.println("Enter a ground distance in inches:");
				aimStep = AimStep::AskDistance;
			} else if (c0 == 'n' || c0 == 'N') {
				Serial.println("Done. Press 'r' to re-home and start over.");
				aimStep = AimStep::Stopped;
			} else {
				Serial.println("Please answer y or n.");
			}
			break;
		}

		case AimStep::Stopped:
			break;
	}
}

void handleSerial() {
	while (Serial.available() > 0) {
		const char c = static_cast<char>(Serial.read());
		if (c == '\r') continue;
		if (c == '\n') {
			inputBuffer.trim();
			if (inputBuffer.length() > 0) handleLine(inputBuffer);
			inputBuffer = "";
			continue;
		}
		inputBuffer += c;
	}
}

void setup() {
	Serial.begin(115200);

	pinMode(Config::X::enablePin, OUTPUT);
	digitalWrite(Config::X::enablePin, LOW);
	motorX.begin();
	motorY.begin();

	Wire.begin(kSdaPin, kSclPin);
	bnoReady = bno.begin();

	Serial.println();
	Serial.println("=== Aim-angle test (home -> neutral -> ready) ===");
	Serial.print("Laser height: ");
	Serial.print(kLaserHeightInches, 1);
	Serial.println(" in");
	Serial.println(bnoReady ? "BNO055 ready." : "BNO055 NOT found (tilt will read 0).");

	homingX.begin();
	homingY.begin();
	homingX.startSeek();
	homingY.startSeek();
	Serial.println("Homing (double-click)...");
}

void loop() {
	SP::poll();
	motorX.update();
	motorY.update();

	switch (phase) {
		case Phase::Homing:
			updateSyncedHoming();
			if (homingPhase == HomingPhase::Done) {
				// Home repeatability: if these vary run-to-run, the home itself is
				// scattering (amplified ~0.2"/microstep on the ground at this height).
				Serial.print("[Repeatability] seek->reapproach steps  X=");
				Serial.print(homingX.reapproachTriggerPos - homingX.seekTriggerPos);
				Serial.print("  Y=");
				Serial.println(homingY.reapproachTriggerPos - homingY.seekTriggerPos);
				motorX.setPositionSteps(0);   // zero at endstop home
				motorY.setPositionSteps(0);
				Serial.println("Homed & zeroed. Moving to neutral...");
				toNeutral.startTo(Config::Neutral::x, Config::Neutral::y,
						Config::Z::travelSpeedStepsPerSec);
				phase = Phase::ToNeutral;
			}
			break;

		case Phase::ToNeutral:
			toNeutral.update();
			if (!toNeutral.busy()) {
				Serial.println();
				Serial.println("At neutral. Ready.");
				Serial.println("Enter a ground distance in inches:");
				aimStep = AimStep::AskDistance;
				phase = Phase::Ready;
			}
			break;

		case Phase::Ready:
			handleSerial();
			aimX.update();
			// When the aim move finishes, prompt for the measured distance. Gate on
			// the Moving state (not the aiming flag) so it can't be skipped.
			if (aimStep == AimStep::Moving && !aimX.busy()) {
				aiming = false;
				Serial.println("[aim] X at target angle.");
				Serial.println();
				Serial.println("Where did it actually hit? Enter the measured distance (inches):");
				aimStep = AimStep::AskMeasured;
			}
			break;
	}
}
