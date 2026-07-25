// Home X/Y in lockstep (seek / backoff / reapproach), zero together, then manual jog.

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Endstop.h"

enum class AppPhase : uint8_t { Homing, Manual };
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

	// During Seek: wait until pressed, then stop and mark phaseDone.
	void updateSeek() {
		endstop.update();
		if (phaseDone) return;

		if (endstop.isPressed()) {
			stopMotor();
			phaseDone = true;
			Serial.print('[');
			Serial.print(name);
			Serial.println("] endstop pressed (1st) — waiting for other axis");
		}
	}

	// During Backoff: move away until released, then a fixed margin further, then stop.
	void updateBackoff() {
		endstop.update();
		if (phaseDone) return;

		// Phase 1: keep moving away until the switch reports released, then latch a
		// target a fixed margin further in the away direction.
		if (!backoffReleased) {
			if (!endstop.isPressed()) {
				backoffReleased = true;
				const bool away = (motor.direction() == Motor::Direction::Forward);
				backoffTargetPos = motor.positionSteps()
						+ (away ? -Config::Z::backoffMarginSteps : Config::Z::backoffMarginSteps);
			}
			return;
		}

		// Phase 2: keep going until the margin is covered (>= handles overshoot, no hang).
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

	// During Reapproach: wait until pressed, then stop and mark phaseDone.
	void updateReapproach() {
		endstop.update();
		if (phaseDone) return;

		if (endstop.isPressed()) {
			stopMotor();
			phaseDone = true;
			Serial.print('[');
			Serial.print(name);
			Serial.println("] endstop pressed (2nd) — waiting for other axis");
		}
	}
};

AxisHoming homingX{motorX, endstopX, "X"};
AxisHoming homingY{motorY, endstopY, "Y"};
HomingPhase homingPhase = HomingPhase::Seek;

enum class ManualAxis : uint8_t { X, Y };

String inputBuffer;
AppPhase appPhase = AppPhase::Homing;
bool commandMovePending = false;

long totalX = 0;
long totalY = 0;

struct ManualLeg {
	Motor& motor;
	const char* name;
	long targetPos = 0;
	bool active = false;

	explicit ManualLeg(Motor& motorIn, const char* nameIn) : motor(motorIn), name(nameIn) {}

	void start(long deltaSteps) {
		if (deltaSteps == 0) return;

		const long startPos = motor.positionSteps();
		// Direction flipped: positive delta now drives Forward, negative drives Reverse.
		// Motor counts Reverse as +1 step and Forward as -1, so the target must mirror the
		// flipped direction (startPos - delta) for the move to complete.
		targetPos = startPos - deltaSteps;
		active = true;

		motor.enable(true);
		motor.setDirection(deltaSteps > 0 ? Motor::Direction::Forward : Motor::Direction::Reverse);
		motor.setSpeedStepsPerSec(Config::manualSpeedStepsPerSec);

		Serial.print("[Move ");
		Serial.print(name);
		Serial.print("] From ");
		Serial.print(startPos);
		Serial.print(" to ");
		Serial.print(targetPos);
		Serial.print(" (delta ");
		Serial.print(deltaSteps);
		Serial.println(")");
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
		Serial.println("] move complete");
	}
};

ManualLeg legX{motorX, "X"};
ManualLeg legY{motorY, "Y"};

ManualLeg& legForAxis(ManualAxis axis) {
	return (axis == ManualAxis::Y) ? legY : legX;
}

long& totalForAxis(ManualAxis axis) {
	return (axis == ManualAxis::Y) ? totalY : totalX;
}

void printTotals() {
	Serial.print("[Total X] ");
	Serial.println(totalX);
	Serial.print("[Total Y] ");
	Serial.println(totalY);
}

bool parseAxisChar(char c, ManualAxis& outAxis) {
	switch (c) {
		case 'X':
		case 'x':
			outAxis = ManualAxis::X;
			return true;
		case 'Y':
		case 'y':
			outAxis = ManualAxis::Y;
			return true;
		default:
			return false;
	}
}

bool parseSignedLongFrom(const String& text, size_t& index, long& outValue) {
	while (index < text.length() && isspace(static_cast<unsigned char>(text.charAt(index)))) {
		index++;
	}
	if (index >= text.length()) return false;

	const size_t start = index;
	if (text.charAt(index) == '+' || text.charAt(index) == '-') {
		index++;
	}
	while (index < text.length() && isdigit(static_cast<unsigned char>(text.charAt(index)))) {
		index++;
	}
	if (index == start || (index == start + 1 && !isdigit(static_cast<unsigned char>(text.charAt(start))))) {
		return false;
	}

	const String numPart = text.substring(start, index);
	char* endPtr = nullptr;
	outValue = strtol(numPart.c_str(), &endPtr, 10);
	return endPtr != nullptr && *endPtr == '\0';
}

void handleLine(const String& line) {
	size_t index = 0;
	bool anyCommand = false;
	bool anyMove = false;

	while (index < line.length()) {
		while (index < line.length() && isspace(static_cast<unsigned char>(line.charAt(index)))) {
			index++;
		}
		if (index >= line.length()) break;

		const char axisChar = line.charAt(index);
		ManualAxis axis;
		if (!parseAxisChar(axisChar, axis)) {
			Serial.println("Unknown command. Use X100, X-110, Y 20, or X 10 Y 20");
			return;
		}
		index++;

		long steps = 0;
		if (!parseSignedLongFrom(line, index, steps)) {
			Serial.println("Unknown command. Use X100, X-110, Y 20, or X 10 Y 20");
			return;
		}

		totalForAxis(axis) += steps;
		legForAxis(axis).start(steps);
		anyCommand = true;
		if (steps != 0) anyMove = true;
	}

	if (!anyCommand) {
		Serial.println("Unknown command. Use X100, X-110, Y 20, or X 10 Y 20");
		return;
	}

	if (anyMove) commandMovePending = true;
	else printTotals();
}

void handleSerial() {
	while (Serial.available() > 0) {
		const char c = static_cast<char>(Serial.read());
		if (c == '\n' || c == '\r') {
			if (inputBuffer.length() == 0) continue;
			inputBuffer.trim();
			handleLine(inputBuffer);
			inputBuffer = "";
			continue;
		}

		inputBuffer += c;
	}
}

void updateManualMoves() {
	legX.update();
	legY.update();

	if (!commandMovePending) return;
	if (legX.active || legY.active) return;

	commandMovePending = false;
	printTotals();
}

void enterManualPhase() {
	motorX.setPositionSteps(0);
	motorY.setPositionSteps(0);

	totalX = 0;
	totalY = 0;
	appPhase = AppPhase::Manual;
	homingPhase = HomingPhase::Done;

	Serial.println();
	Serial.println("=== Homing complete — X and Y zeroed together ===");
	Serial.println("Manual commands: X100, X-110, X 10 Y 20");
	Serial.print("Speed (steps/s): ");
	Serial.println(Config::manualSpeedStepsPerSec);
	printTotals();
}

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

void updateSyncedHoming() {
	switch (homingPhase) {
		case HomingPhase::Seek:
			homingX.updateSeek();
			homingY.updateSeek();
			if (homingX.phaseDone && homingY.phaseDone) {
				startBackoffPhase();
			}
			break;

		case HomingPhase::Backoff:
			homingX.updateBackoff();
			homingY.updateBackoff();
			if (homingX.phaseDone && homingY.phaseDone) {
				startReapproachPhase();
			}
			break;

		case HomingPhase::Reapproach:
			homingX.updateReapproach();
			homingY.updateReapproach();
			if (homingX.phaseDone && homingY.phaseDone) {
				Serial.println("[XY] both pressed (2nd) — homed");
				enterManualPhase();
			}
			break;

		case HomingPhase::Done:
			break;
	}
}

void setup() {
	Serial.begin(115200);

	pinMode(Config::X::enablePin, OUTPUT);
	digitalWrite(Config::X::enablePin, LOW);

	motorX.begin();
	motorY.begin();

	Serial.println();
	Serial.println("=== Zero motors test (X/Y synced homing) ===");
	Serial.println("X and Y start/advance/finish each phase together.");

	homingX.begin();
	homingY.begin();
	startSeekPhase();
}

void loop() {
	motorX.update();
	motorY.update();

	if (appPhase == AppPhase::Homing) {
		updateSyncedHoming();
		return;
	}

	handleSerial();
	updateManualMoves();
}
