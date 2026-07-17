// Standalone endstop monitor: X, Y, Z using Config pins (NO + INPUT_PULLUP).

#include <Arduino.h>
#include "Config.h"
#include "Endstop.h"

namespace {

Endstop endstopX(Config::X::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);
Endstop endstopY(Config::Y::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);
Endstop endstopZ(Config::Z::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);

struct AxisEndstop {
	Endstop& endstop;
	const char* name;
	uint8_t pin;
	bool lastPressed;

	explicit AxisEndstop(Endstop& endstopIn, const char* nameIn, uint8_t pinIn)
		: endstop(endstopIn), name(nameIn), pin(pinIn), lastPressed(false) {}

	void begin() {
		endstop.begin();
		lastPressed = endstop.isPressed();
	}

	void update() {
		endstop.update();
		const bool pressed = endstop.isPressed();
		if (pressed == lastPressed) return;

		Serial.print('[');
		Serial.print(name);
		Serial.print(" endstop GPIO ");
		Serial.print(pin);
		Serial.print("] ");
		Serial.println(pressed ? "PRESSED" : "RELEASED");

		lastPressed = pressed;
	}

	void printInitial() const {
		Serial.print('[');
		Serial.print(name);
		Serial.print(" endstop GPIO ");
		Serial.print(pin);
		Serial.print("] ");
		Serial.println(lastPressed ? "PRESSED (initial)" : "RELEASED (initial)");
	}
};

AxisEndstop axisX{endstopX, "X", Config::X::endstopPin};
AxisEndstop axisY{endstopY, "Y", Config::Y::endstopPin};
AxisEndstop axisZ{endstopZ, "Z", Config::Z::endstopPin};

} // namespace

void setup() {
	Serial.begin(115200);
	delay(100); 

	axisX.begin();
	axisY.begin();
	axisZ.begin();

	Serial.println();
	Serial.println("=== Endstop monitor (NO + INPUT_PULLUP) ===");
	Serial.print("Debounce (ms): ");
	Serial.println(Config::endstopDebounceMs);
	Serial.println("Press each switch to verify PRESSED / RELEASED on serial.");
	Serial.println();

	axisX.printInitial();
	axisY.printInitial();
	axisZ.printInitial();
}

void loop() {
	axisX.update();
	axisY.update();
	axisZ.update();
}
