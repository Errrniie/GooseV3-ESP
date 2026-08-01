// PatternShot: home (double-click) -> zero -> move to neutral -> then TYPE a
// near/far distance in the terminal and it builds a ground box and walks X/Y
// around it, repeating N times. Type new distances anytime to re-shoot.

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Endstop.h"
#include "AxisMoveTo.h"
#include "PatternShot.h"
#include "SP_System.h"

// Laser height above the ground (6 ft 11 in) -- used to turn a ground distance
// into an X aim coordinate, same geometry as the aim test.
static constexpr float kLaserHeightInches = 83.0f;
static constexpr float kBoxHalfDepthInches = 10.0f;   // default near/far spread around center
static constexpr long kBoxHalfWidth = 100;            // default Y half-width (steps)

// Speed the box is traced at. There is NO acceleration ramp, so the motor jumps
// straight to this rate -- push it too high and the closed-loop driver can throw
// a following-error fault (red) or lose steps. Raise gradually.
static constexpr float kPatternSpeedStepsPerSec = 30.0f * Config::Z::travelSpeedStepsPerSec;  // 3x travel

Motor motorX(Config::X::stepPin, Config::X::dirPin);
Motor motorY(Config::Y::stepPin, Config::Y::dirPin);
Endstop endstopX(Config::X::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);
Endstop endstopY(Config::Y::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);

MoveXY toNeutral(motorX, motorY);
PatternShot pattern(motorX, motorY);

PatternCorner boxCorners[PatternShot::kNumCorners];   // filled from terminal input
static constexpr uint16_t kRepeats = 30;

// Ground distance (in) -> X aim coordinate. Higher X = farther (toward neutral
// 655 = horizontal). Same convention as the aim test.
long xForDistance(float distanceInches) {
	const float beta = atanf(kLaserHeightInches / distanceInches) * 180.0f / PI;  // depression
	return Config::Neutral::x - Config::stepsForDegrees(beta);
}

// ---------------- Synced X/Y homing (seek -> backoff -> reapproach) ----------
enum class HomingPhase : uint8_t { Seek, Backoff, Reapproach, Done };

struct AxisHoming {
	Motor& motor;
	Endstop& endstop;
	const char* name;
	bool phaseDone = false;
	bool backoffReleased = false;
	long backoffTargetPos = 0;

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

AxisHoming homingX{motorX, endstopX, "X"};
AxisHoming homingY{motorY, endstopY, "Y"};
HomingPhase homingPhase = HomingPhase::Seek;

enum class Phase : uint8_t { Homing, ToNeutral, AwaitInput, Pattern };
Phase phase = Phase::Homing;
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

void promptForBox() {
	Serial.println();
	Serial.println("Enter a CENTER distance (in) to build a box around:");
	Serial.println("  <center_in> [half_depth_in] [half_width_steps]");
	Serial.println("  e.g.  400        (box 390..410 in, default width)");
	Serial.println("  e.g.  400 25     (box 375..425 in)");
	Serial.println("  e.g.  400 25 150 (deeper + wider box)");
}

// Parse "center [half_depth_in] [half_width_steps]" -> box AROUND the center ->
// start the pattern. near = center - half_depth, far = center + half_depth.
void startBoxFromLine(const String& line) {
	float centerD = 0, halfDepthF = 0, halfWidthF = 0;
	const int n = sscanf(line.c_str(), "%f %f %f", &centerD, &halfDepthF, &halfWidthF);
	if (n < 1 || centerD <= 0) {
		Serial.println("Need a positive center distance, e.g.  400");
		return;
	}
	const float halfDepth = (n >= 2 && halfDepthF > 0) ? halfDepthF : kBoxHalfDepthInches;
	const long halfW = (n >= 3 && halfWidthF > 0) ? (long)halfWidthF : kBoxHalfWidth;

	const float nearD = centerD - halfDepth;
	const float farD = centerD + halfDepth;
	if (nearD <= 0) {
		Serial.println("Center too close for that depth (near edge <= 0). Use a bigger center.");
		return;
	}

	const long nearX = xForDistance(nearD);
	const long farX = xForDistance(farD);
	const long yLo = Config::Neutral::y - halfW;
	const long yHi = Config::Neutral::y + halfW;

	boxCorners[0] = { nearX, yLo };
	boxCorners[1] = { nearX, yHi };
	boxCorners[2] = { farX,  yHi };
	boxCorners[3] = { farX,  yLo };

	Serial.println();
	Serial.print("Box around ");
	Serial.print(centerD, 0);
	Serial.print("in:  near ");
	Serial.print(nearD, 0);
	Serial.print("in (X ");
	Serial.print(nearX);
	Serial.print(")  far ");
	Serial.print(farD, 0);
	Serial.print("in (X ");
	Serial.print(farX);
	Serial.print(")  Y ");
	Serial.print(yLo);
	Serial.print("..");
	Serial.println(yHi);

	pattern.configure(boxCorners, kRepeats, kPatternSpeedStepsPerSec);
	pattern.begin();
	phase = Phase::Pattern;
}

void handleSerial() {
	while (Serial.available() > 0) {
		const char c = static_cast<char>(Serial.read());
		if (c == '\r') continue;
		if (c == '\n') {
			inputBuffer.trim();
			if (inputBuffer.length() > 0) startBoxFromLine(inputBuffer);
			inputBuffer = "";
			continue;
		}
		inputBuffer += c;
	}
}

void setup() {
	Serial.begin(115200);

	pinMode(Config::X::enablePin, OUTPUT);
	// Clear any latched driver alarm (prior following-error fault): disable, pause,
	// re-enable. A hard latch still needs a physical driver power-cycle.
	digitalWrite(Config::X::enablePin, HIGH);   // disable
	delay(600);
	digitalWrite(Config::X::enablePin, LOW);    // enable
	delay(200);

	motorX.begin();
	motorY.begin();

	Serial.println();
	Serial.println("=== PatternShot (home -> neutral -> type a box) ===");

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
				Serial.println("At neutral.");
				promptForBox();
				phase = Phase::AwaitInput;
			}
			break;

		case Phase::AwaitInput:
			handleSerial();   // waits for "<near> <far> [width]" then starts the pattern
			break;

		case Phase::Pattern:
			pattern.update();
			if (pattern.isDone()) {
				Serial.print("=== Pattern complete: ");
				Serial.print(pattern.repeatsTotal());
				Serial.println(" loops. ===");
				promptForBox();
				phase = Phase::AwaitInput;
			}
			break;
	}
}
