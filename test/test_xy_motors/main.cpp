// Serial jog test for X and Y steppers.
// Commands: x12, x-12, y50, y-3  (axis letter + signed step count)

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"

namespace {

Motor motorX(Config::X::stepPin, Config::X::dirPin);
Motor motorY(Config::Y::stepPin, Config::Y::dirPin);

struct AxisJog {
	Motor& motor;
	char name;
	long targetPos = 0;
	bool active = false;

	explicit AxisJog(Motor& motorIn, char nameIn) : motor(motorIn), name(nameIn) {}

	void start(long deltaSteps) {
		if (deltaSteps == 0) {
			Serial.print(name);
			Serial.println(": 0 steps (no move)");
			return;
		}

		const long startPos = motor.positionSteps();
		targetPos = startPos + deltaSteps;
		active = true;

		motor.enable(true);
		motor.setDirection(deltaSteps > 0 ? Motor::Direction::Reverse : Motor::Direction::Forward);
		motor.setSpeedStepsPerSec(Config::manualSpeedStepsPerSec);

		Serial.print('[');
		Serial.print(name);
		Serial.print("] ");
		Serial.print(startPos);
		Serial.print(" -> ");
		Serial.print(targetPos);
		Serial.print(" (");
		Serial.print(deltaSteps > 0 ? '+' : '\0');
		Serial.print(deltaSteps);
		Serial.println(" steps)");
	}

	void update() {
		if (!active) return;

		const long pos = motor.positionSteps();
		const bool forward = (motor.direction() == Motor::Direction::Forward);
		const bool done = forward ? (pos <= targetPos) : (pos >= targetPos);

		if (done) {
			motor.setSpeedStepsPerSec(0);
			active = false;
			Serial.print('[');
			Serial.print(name);
			Serial.print("] done at ");
			Serial.println(pos);
		}
	}
};

AxisJog jogX{motorX, 'X'};
AxisJog jogY{motorY, 'Y'};

String inputBuffer;

void setupSharedEnable() {
	pinMode(Config::X::enablePin, OUTPUT);
	// Active-low drivers: LOW = enabled
	digitalWrite(Config::X::enablePin, LOW);
}

bool parseAxisCommand(const String& line, char& axisOut, long& stepsOut) {
	if (line.length() < 2) return false;

	const char axis = static_cast<char>(tolower(static_cast<unsigned char>(line.charAt(0))));
	if (axis != 'x' && axis != 'y') return false;

	String numPart = line.substring(1);
	numPart.trim();
	if (numPart.length() == 0) return false;

	char* endPtr = nullptr;
	const long steps = strtol(numPart.c_str(), &endPtr, 10);
	if (endPtr == nullptr || *endPtr != '\0') return false;

	axisOut = axis;
	stepsOut = steps;
	return true;
}

void handleLine(const String& line) {
	char axis = 0;
	long steps = 0;
	if (!parseAxisCommand(line, axis, steps)) {
		Serial.println("Unknown command. Use x12, x-12, y50, y-3");
		return;
	}

	if (axis == 'x') {
		jogX.start(steps);
	} else {
		jogY.start(steps);
	}
}

} // namespace

void setup() {
	Serial.begin(115200);
	delay(500);

	setupSharedEnable();
	motorX.begin();
	motorY.begin();

	Serial.println();
	Serial.println("=== X/Y motor jog test ===");
	Serial.println("Type axis + signed steps, e.g. x12  x-12  y100  y-5");
	Serial.print("Speed (steps/s): ");
	Serial.println(Config::manualSpeedStepsPerSec);
	Serial.print("X pos=");
	Serial.print(motorX.positionSteps());
	Serial.print("  Y pos=");
	Serial.println(motorY.positionSteps());
}

void loop() {
	while (Serial.available() > 0) {
		const char c = static_cast<char>(Serial.read());
		if (c == '\n' || c == '\r') {
			if (inputBuffer.length() > 0) {
				inputBuffer.trim();
				handleLine(inputBuffer);
				inputBuffer = "";
			}
		} else {
			inputBuffer += c;
		}
	}

	motorX.update();
	motorY.update();
	jogX.update();
	jogY.update();
}
