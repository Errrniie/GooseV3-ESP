#include <Arduino.h>
#include <Endstop.h>
#include <Motor.h>
#include <Homing.h>
#include <Config.h>
#include <AppState.h>
#include <Search.h>
#include <Tracking.h>
#include <SP_System.h>

Motor motorZ(Config::Z::stepPin, Config::Z::dirPin);
Motor motorX(Config::X::stepPin, Config::X::dirPin);
Motor motorY(Config::Y::stepPin, Config::Y::dirPin);
Endstop endstop(Config::Z::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);

Homing::Config homingCfg = [] {
	Homing::Config cfg;
	cfg.homeDir = Config::Z::homeDir;
	cfg.seekSpeedStepsPerSec = Config::Z::seekSpeedStepsPerSec;
	cfg.backoffSpeedStepsPerSec = Config::Z::backoffSpeedStepsPerSec;
	cfg.backoffMarginSteps = Config::Z::backoffMarginSteps;
	cfg.reapproachSpeedStepsPerSec = Config::Z::reapproachSpeedStepsPerSec;
	cfg.travelSpeedStepsPerSec = Config::Z::travelSpeedStepsPerSec;
	cfg.finalPositionSteps = Config::Z::finalPositionSteps;
	cfg.finalMoveTimeoutMs = Config::Z::finalMoveTimeoutMs;
	return cfg;
}();

Homing homing(motorZ, endstop, homingCfg);

Search::Config searchCfg = [] {
	Search::Config cfg;
	cfg.minSteps = Config::searchMinSteps;
	cfg.maxSteps = Config::searchMaxSteps;
	cfg.speedStepsPerSec = Config::searchSpeedStepsPerSec;
	return cfg;
}();

Search search(searchCfg);

AppState appState = AppState::Homing;
bool transitionedAfterHoming = false;

Homing::State lastState = Homing::State::Idle;

enum class ManualAxis : uint8_t { None, Z, X, Y };

String inputBuffer;
bool manualReadyPrinted = false;
ManualAxis manualAxis = ManualAxis::None;
long manualTargetPos = 0;
long manualLegStartPos = 0;
long totalManualMovementSteps = 0;

Motor& motorForAxis(ManualAxis axis) {
	switch (axis) {
		case ManualAxis::X: return motorX;
		case ManualAxis::Y: return motorY;
		default: return motorZ;
	}
}

const char* manualAxisName(ManualAxis axis) {
	switch (axis) {
		case ManualAxis::X: return "X";
		case ManualAxis::Y: return "Y";
		case ManualAxis::Z: return "Z";
		default: return "?";
	}
}

void addManualTravelToTotal(long fromPos, long toPos) {
	totalManualMovementSteps += (toPos - fromPos);
}

bool manualMovesAllowed() {
	return homing.isHomed()
			&& (appState != AppState::Searching || search.isPaused());
}

void printManualPrompt() {
	Serial.println("Manual mode:");
	Serial.println("  Z: signed steps (ex: 200 or -200)");
	Serial.println("  X: X10 or X-10");
	Serial.println("  Y: Y10 or Y-10");
	if (Config::afterHomingManualTrackingTest) {
		Serial.println("Type 'd' to stop (partial Z move updates total by signed distance).");
	} else {
		Serial.println("Type 'd' to stop motion. When search is paused, 'c' resumes sweep. 'r' resets the ESP.");
	}
	Serial.print("degreesPerStep=");
	Serial.println(Config::degreesPerStep, 6);
	Serial.print("[Total movement] ");
	Serial.println(totalManualMovementSteps);
}

bool parseSignedLong(const String& line, long& outValue) {
	char* endPtr = nullptr;
	outValue = strtol(line.c_str(), &endPtr, 10);
	return endPtr != nullptr && *endPtr == '\0';
}

bool parseAxisMove(const String& line, ManualAxis& outAxis, long& outSteps) {
	if (line.length() < 2) return false;

	const char lead = line.charAt(0);
	if (lead == 'X' || lead == 'x') {
		outAxis = ManualAxis::X;
	} else if (lead == 'Y' || lead == 'y') {
		outAxis = ManualAxis::Y;
	} else {
		return false;
	}

	return parseSignedLong(line.substring(1), outSteps);
}

void stopMotion(const char* reason) {
	if (manualAxis == ManualAxis::Z && Config::afterHomingManualTrackingTest
			&& appState == AppState::Tracking) {
		addManualTravelToTotal(manualLegStartPos, motorZ.positionSteps());
		Serial.print("[Total movement] ");
		Serial.println(totalManualMovementSteps);
	}

	motorZ.setSpeedStepsPerSec(0);
	motorX.setSpeedStepsPerSec(0);
	motorY.setSpeedStepsPerSec(0);
	manualAxis = ManualAxis::None;

	if (reason) {
		Serial.print("[Stop] ");
		Serial.println(reason);
	}
}

void startManualMove(ManualAxis axis, long deltaSteps) {
	if (deltaSteps == 0) {
		Serial.println("0 steps does nothing.");
		return;
	}

	Motor& motor = motorForAxis(axis);
	const long startPos = motor.positionSteps();
	manualLegStartPos = startPos;
	manualTargetPos = startPos + deltaSteps;
	manualAxis = axis;

	motor.enable(true);
	motor.setDirection(deltaSteps > 0 ? Motor::Direction::Reverse : Motor::Direction::Forward);
	motor.setSpeedStepsPerSec(Config::manualSpeedStepsPerSec);

	Serial.print("[Move ");
	Serial.print(manualAxisName(axis));
	Serial.print("] From ");
	Serial.print(startPos);
	Serial.print(" to ");
	Serial.print(manualTargetPos);
	Serial.print(" (delta ");
	Serial.print(deltaSteps);
	Serial.println(")");
}

void updateManualMove() {
	if (manualAxis == ManualAxis::None) return;

	Motor& motor = motorForAxis(manualAxis);
	const long pos = motor.positionSteps();
	const bool forward = (motor.direction() == Motor::Direction::Forward);
	const bool done = forward ? (pos <= manualTargetPos) : (pos >= manualTargetPos);

	if (!done) return;

	if (manualAxis == ManualAxis::Z && Config::afterHomingManualTrackingTest
			&& appState == AppState::Tracking) {
		addManualTravelToTotal(manualLegStartPos, motor.positionSteps());
	}

	motor.setSpeedStepsPerSec(0);
	const ManualAxis finishedAxis = manualAxis;
	manualAxis = ManualAxis::None;

	Serial.print("[Stop ");
	Serial.print(manualAxisName(finishedAxis));
	Serial.println("] move complete");
	Serial.print("[Pos ");
	Serial.print(manualAxisName(finishedAxis));
	Serial.print("] ");
	Serial.println(motor.positionSteps());

	if (finishedAxis == ManualAxis::Z && Config::afterHomingManualTrackingTest
			&& appState == AppState::Tracking) {
		Serial.print("[Total movement] ");
		Serial.println(totalManualMovementSteps);
	}

	if (manualMovesAllowed()) {
		printManualPrompt();
	}
}

void handleSerial() {
	while (Serial.available() > 0) {
		const char c = static_cast<char>(Serial.read());

		if (c == '\r') continue;

		if (c == 'd' || c == 'D') {
			stopMotion("STOP (d) received");
			if (!homing.isHomed()) {
				homing.abort("stopped by user");
			}
			if (appState == AppState::Searching) {
				search.pause(motorZ);
			}
			inputBuffer = "";
			continue;
		}

		// 'r'/'R' is reserved for software reset (SP::poll); 'c' resumes the sweep.
		if (c == 'c' || c == 'C') {
			if (appState == AppState::Searching && search.isPaused() && manualAxis == ManualAxis::None) {
				search.resume(motorZ);
				Serial.println("[Search] Resumed.");
			}
			inputBuffer = "";
			continue;
		}

		if (c == '\n') {
			inputBuffer.trim();
			const String line = inputBuffer;
			inputBuffer = "";

			if (!homing.isHomed()) {
				Serial.println("Not homed yet. Waiting for homing to finish...");
				continue;
			}

			if (appState == AppState::Searching && !search.isPaused()) {
				if (line.length() == 0) {
					Serial.println("Search active: motion is sweeping. Type 'd' to pause, then you can jog; 'c' resumes.");
					continue;
				}
				long steps = 0;
				ManualAxis axis = ManualAxis::None;
				if (parseAxisMove(line, axis, steps) || parseSignedLong(line, steps)) {
					Serial.println("Search active: type 'd' to pause before manual moves.");
				} else {
					Serial.println("Invalid input. Type 'd' to pause search first.");
				}
				continue;
			}

			if (manualAxis != ManualAxis::None) {
				Serial.println("Busy moving. Type 'd' to stop.");
				continue;
			}

			if (line.length() == 0) {
				if (manualMovesAllowed()) {
					printManualPrompt();
				}
				continue;
			}

			ManualAxis axis = ManualAxis::None;
			long steps = 0;
			if (parseAxisMove(line, axis, steps)) {
				startManualMove(axis, steps);
				continue;
			}

			if (parseSignedLong(line, steps)) {
				startManualMove(ManualAxis::Z, steps);
				continue;
			}

			Serial.println("Invalid input. Z: 200 or -200. X: X10. Y: Y-10.");
			if (manualMovesAllowed()) {
				printManualPrompt();
			}
			continue;
		}

		inputBuffer += c;
	}
}

void printStateIfChanged() {
	const Homing::State s = homing.state();
	if (s == lastState) return;
	lastState = s;

	Serial.print("[Homing] ");
	Serial.println(homing.stateName());
	if (homing.hasError()) {
		Serial.print("[Homing] Error: ");
		Serial.println(homing.errorReason() ? homing.errorReason() : "(unknown)");
	}
}

void setup() {
	Serial.begin(115200);
	delay(1000);

	pinMode(Config::Z::enablePin, OUTPUT);
	digitalWrite(Config::Z::enablePin, LOW);

	motorZ.begin();
	motorX.begin();
	motorY.begin();
	endstop.begin();

	Serial.println("Starting homing...");
	homing.start();
	printStateIfChanged();
}

void loop() {
	SP::poll();
	handleSerial();
	endstop.update();

	homing.update();
	printStateIfChanged();

	if (appState == AppState::Homing && homing.isHomed() && !transitionedAfterHoming) {
		transitionedAfterHoming = true;
		Serial.println("Homing complete.");
		Serial.print("[Pos Z] ");
		Serial.println(motorZ.positionSteps());
		if (Config::afterHomingManualTrackingTest) {
			appState = AppState::Tracking;
			totalManualMovementSteps = 0;
			Serial.println("Manual tracking test: positive Z steps add to total, negative subtract.");
		} else {
			appState = AppState::Searching;
			search.start(motorZ);
			Serial.println("Entering search sweep (min..max). Type 'd' to pause.");
		}
	}

	if (appState == AppState::Searching) {
		search.update(motorZ);
	} else if (appState == AppState::Tracking) {
		Tracking::update(motorZ);
	}

	if (homing.isHomed() && !manualReadyPrinted && manualMovesAllowed()) {
		manualReadyPrinted = true;
		printManualPrompt();
	}

	motorZ.update();
	motorX.update();
	motorY.update();
	updateManualMove();
}
