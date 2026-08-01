// PatternShot ZIGZAG: home (double-click) -> zero -> move to neutral -> then TYPE
// a center distance, depth, and width. It builds a square around that center and
// sweeps it as a zigzag (serpentine raster): top-left -> top-right, step down,
// right -> left, step down, ... to the bottom corner. Repeats kRepeats loops.

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Endstop.h"
#include "AxisMoveTo.h"
#include "SP_System.h"

static constexpr float kLaserHeightInches = 83.0f;   // 6 ft 11 in above the ground
static constexpr float kDefaultDepthInches = 10.0f;  // X near/far spread around center (+/-)
static constexpr long kDefaultWidthSteps = 100;      // Y spread around neutral (+/-)
static constexpr int kDefaultRows = 7;               // zigzag horizontal passes (odd -> ends bottom-right)
static constexpr uint16_t kRepeats = 100;            // full zigzag loops

// Zigzag trace speed. No acceleration ramp -- raise gradually or the closed-loop
// driver can throw a following-error fault (red). Short segments start/stop a lot.
static constexpr float kZigzagSpeed = 30.0f * Config::Z::travelSpeedStepsPerSec;   // 3x travel

static constexpr int kMaxRows = 20;
static constexpr int kMaxWaypoints = 2 * kMaxRows;

Motor motorX(Config::X::stepPin, Config::X::dirPin);
Motor motorY(Config::Y::stepPin, Config::Y::dirPin);
Endstop endstopX(Config::X::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);
Endstop endstopY(Config::Y::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);

MoveXY mover(motorX, motorY);   // used for both the neutral move and each zigzag segment

struct Waypoint { long x; long y; };
Waypoint waypoints[kMaxWaypoints];
int waypointCount = 0;
int wpIndex = 0;
uint16_t loopsDone = 0;

// Ground distance (in) -> X aim coordinate. Higher X = farther (toward neutral).
long xForDistance(float distanceInches) {
	const float beta = atanf(kLaserHeightInches / distanceInches) * 180.0f / PI;  // depression
	return Config::Neutral::x - Config::stepsForDegrees(beta);
}

// Build the serpentine waypoint list over the square (farX..nearX) x (yLo..yHi).
// Row 0 is the far edge (top): left->right. Each following row steps toward near
// (down) and reverses Y direction. Ends at the bottom-right when rows is odd.
void buildZigzag(long nearX, long farX, long yLo, long yHi, int rows) {
	if (rows < 2) rows = 2;
	if (rows > kMaxRows) rows = kMaxRows;
	waypointCount = 0;
	for (int i = 0; i < rows; ++i) {
		const long x = farX + (long)((float)(nearX - farX) * i / (float)(rows - 1));
		long yA, yB;
		if (i % 2 == 0) { yA = yLo; yB = yHi; }   // left -> right
		else            { yA = yHi; yB = yLo; }   // right -> left
		waypoints[waypointCount++] = { x, yA };
		waypoints[waypointCount++] = { x, yB };
	}
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

enum class Phase : uint8_t { Homing, ToNeutral, AwaitInput, Zigzag };
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

void promptForZigzag() {
	Serial.println();
	Serial.println("Enter a zigzag square:  <center_in> [depth_in] [width_steps] [rows]");
	Serial.println("  e.g.  400            (center 400in, +/-10in deep, default width, 7 rows)");
	Serial.println("  e.g.  400 25 150     (deeper + wider)");
	Serial.println("  e.g.  400 25 150 11  (more zigzag passes)");
}

void startZigzagFromLine(const String& line) {
	float centerD = 0, depthF = 0, widthF = 0, rowsF = 0;
	const int n = sscanf(line.c_str(), "%f %f %f %f", &centerD, &depthF, &widthF, &rowsF);
	if (n < 1 || centerD <= 0) {
		Serial.println("Need a positive center distance, e.g.  400");
		return;
	}
	const float depth = (n >= 2 && depthF > 0) ? depthF : kDefaultDepthInches;
	const long width = (n >= 3 && widthF > 0) ? (long)widthF : kDefaultWidthSteps;
	const int rows = (n >= 4 && rowsF >= 2) ? (int)rowsF : kDefaultRows;

	const float nearD = centerD - depth;
	const float farD = centerD + depth;
	if (nearD <= 0) {
		Serial.println("Center too close for that depth (near edge <= 0). Use a bigger center.");
		return;
	}

	const long nearX = xForDistance(nearD);
	const long farX = xForDistance(farD);
	const long yLo = Config::Neutral::y - width;
	const long yHi = Config::Neutral::y + width;

	buildZigzag(nearX, farX, yLo, yHi, rows);

	Serial.println();
	Serial.print("Zigzag around ");
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
	Serial.print(yHi);
	Serial.print("  rows ");
	Serial.print(waypointCount / 2);
	Serial.print("  loops ");
	Serial.println(kRepeats);

	wpIndex = 0;
	loopsDone = 0;
	mover.startTo(waypoints[0].x, waypoints[0].y, kZigzagSpeed);
	phase = Phase::Zigzag;
}

void handleSerial() {
	while (Serial.available() > 0) {
		const char c = static_cast<char>(Serial.read());
		if (c == '\r') continue;
		if (c == '\n') {
			inputBuffer.trim();
			if (inputBuffer.length() > 0) startZigzagFromLine(inputBuffer);
			inputBuffer = "";
			continue;
		}
		inputBuffer += c;
	}
}

void setup() {
	Serial.begin(115200);

	pinMode(Config::X::enablePin, OUTPUT);
	// Clear any latched driver alarm: disable, pause, re-enable.
	digitalWrite(Config::X::enablePin, HIGH);   // disable
	delay(600);
	digitalWrite(Config::X::enablePin, LOW);    // enable
	delay(200);

	motorX.begin();
	motorY.begin();

	Serial.println();
	Serial.println("=== PatternShot ZIGZAG (home -> neutral -> type a square) ===");

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
				mover.startTo(Config::Neutral::x, Config::Neutral::y,
						Config::Z::travelSpeedStepsPerSec);
				phase = Phase::ToNeutral;
			}
			break;

		case Phase::ToNeutral:
			mover.update();
			if (!mover.busy()) {
				Serial.println("At neutral.");
				promptForZigzag();
				phase = Phase::AwaitInput;
			}
			break;

		case Phase::AwaitInput:
			handleSerial();
			break;

		case Phase::Zigzag:
			mover.update();
			if (mover.busy()) break;
			// Reached waypoints[wpIndex]; advance.
			++wpIndex;
			if (wpIndex >= waypointCount) {
				++loopsDone;
				if (loopsDone >= kRepeats) {
					Serial.print("=== Zigzag complete: ");
					Serial.print(kRepeats);
					Serial.println(" loops. ===");
					promptForZigzag();
					phase = Phase::AwaitInput;
					break;
				}
				wpIndex = 0;   // restart from the top for the next loop
			}
			mover.startTo(waypoints[wpIndex].x, waypoints[wpIndex].y, kZigzagSpeed);
			break;
	}
}
