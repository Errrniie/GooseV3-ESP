// Standalone manual movement: X/Y/Z jog via serial (single or combined moves).

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"

Motor motorX(Config::X::stepPin, Config::X::dirPin);
Motor motorY(Config::Y::stepPin, Config::Y::dirPin);
Motor motorZ(Config::Z::stepPin, Config::Z::dirPin);

enum class ManualAxis : uint8_t { X, Y, Z };

String inputBuffer;

struct ManualLeg {
	Motor& motor;
	const char* name;
	long targetPos = 0;
	bool active = false;

	explicit ManualLeg(Motor& motorIn, const char* nameIn) : motor(motorIn), name(nameIn) {}

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
		Serial.print("[Pos ");
		Serial.print(name);
		Serial.print("] ");
		Serial.println(pos);
	}
};

ManualLeg legX{motorX, "X"};
ManualLeg legY{motorY, "Y"};
ManualLeg legZ{motorZ, "Z"};

ManualLeg& legForAxis(ManualAxis axis) {
	switch (axis) {
		case ManualAxis::Y: return legY;
		case ManualAxis::Z: return legZ;
		default: return legX;
	}
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
		case 'Z':
		case 'z':
			outAxis = ManualAxis::Z;
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

	while (index < line.length()) {
		while (index < line.length() && isspace(static_cast<unsigned char>(line.charAt(index)))) {
			index++;
		}
		if (index >= line.length()) break;

		const char axisChar = line.charAt(index);
		ManualAxis axis;
		if (!parseAxisChar(axisChar, axis)) {
			Serial.println("Unknown command. Use X100, X-100, Y 50, Z -3, or X 100 Y 100 Z 100");
			return;
		}
		index++;

		long steps = 0;
		if (!parseSignedLongFrom(line, index, steps)) {
			Serial.println("Unknown command. Use X100, X-100, Y 50, Z -3, or X 100 Y 100 Z 100");
			return;
		}

		legForAxis(axis).start(steps);
		anyCommand = true;
	}

	if (!anyCommand) {
		Serial.println("Unknown command. Use X100, X-100, Y 50, Z -3, or X 100 Y 100 Z 100");
	}
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

void setup() {
	Serial.begin(115200);

	pinMode(Config::X::enablePin, OUTPUT);
	digitalWrite(Config::X::enablePin, LOW);

	motorX.begin();
	motorY.begin();
	motorZ.begin();

	Serial.println();
	Serial.println("=== Manual movement test ===");
	Serial.println("Commands (line-terminated):");
	Serial.println("  X100   X-100   Y50   Z-3");
	Serial.println("  X 100 Y 100 Z 100   (all axes at once)");
	Serial.print("Speed (steps/s): ");
	Serial.println(Config::manualSpeedStepsPerSec);
	Serial.print("X pos=");
	Serial.print(motorX.positionSteps());
	Serial.print("  Y pos=");
	Serial.print(motorY.positionSteps());
	Serial.print("  Z pos=");
	Serial.println(motorZ.positionSteps());
}

void loop() {
	handleSerial();
	motorX.update();
	motorY.update();
	motorZ.update();
	legX.update();
	legY.update();
	legZ.update();
}
