// Standalone homing test: seek → backoff (unclick) → reapproach (second click) → homed.

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"
#include "Endstop.h"

Motor motorX(Config::X::stepPin, Config::X::dirPin);
Motor motorY(Config::Y::stepPin, Config::Y::dirPin);
Motor motorZ(Config::Z::stepPin, Config::Z::dirPin);

Endstop endstopX(Config::X::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);
Endstop endstopY(Config::Y::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);
Endstop endstopZ(Config::Z::endstopPin, Config::endstopDebounceMs, Config::endstopPressedIsPinLow);

enum class HomingPhase : uint8_t { Seek, Backoff, Reapproach, Done };

struct AxisHoming {
	Motor& motor;
	Endstop& endstop;
	const char* name;
	HomingPhase phase = HomingPhase::Seek;
	bool active = false;
	bool homed = false;

	explicit AxisHoming(Motor& motorIn, Endstop& endstopIn, const char* nameIn)
		: motor(motorIn), endstop(endstopIn), name(nameIn) {}

	Motor::Direction awayDir() const {
		return (Config::Z::homeDir == Motor::Direction::Forward)
				? Motor::Direction::Reverse
				: Motor::Direction::Forward;
	}

	void enterSeek() {
		phase = HomingPhase::Seek;
		motor.enable(true);
		motor.setDirection(Config::Z::homeDir);
		motor.setSpeedStepsPerSec(Config::Z::seekSpeedStepsPerSec);
	}

	void enterBackoff() {
		phase = HomingPhase::Backoff;
		motor.enable(true);
		motor.setDirection(awayDir());
		motor.setSpeedStepsPerSec(Config::Z::backoffSpeedStepsPerSec);
	}

	void enterReapproach() {
		phase = HomingPhase::Reapproach;
		motor.enable(true);
		motor.setDirection(Config::Z::homeDir);
		motor.setSpeedStepsPerSec(Config::Z::reapproachSpeedStepsPerSec);
	}

	void finishHomed() {
		motor.setSpeedStepsPerSec(0);
		phase = HomingPhase::Done;
		active = false;
		homed = true;

		Serial.print('[');
		Serial.print(name);
		Serial.print("] homed at ");
		Serial.println(motor.positionSteps());
	}

	void begin() {
		endstop.begin();
		active = true;
		homed = false;
		motor.enable(true);

		Serial.print('[');
		Serial.print(name);
		Serial.println("] homing started");

		if (endstop.isPressed()) {
			Serial.print('[');
			Serial.print(name);
			Serial.println("] endstop already pressed — backing off");
			enterBackoff();
			return;
		}

		enterSeek();
	}

	void update() {
		endstop.update();
		if (!active || homed) return;

		const bool pressed = endstop.isPressed();

		switch (phase) {
			case HomingPhase::Seek:
				if (pressed) {
					Serial.print('[');
					Serial.print(name);
					Serial.println("] endstop pressed — backing off");
					enterBackoff();
				}
				break;

			case HomingPhase::Backoff:
				if (!pressed) {
					Serial.print('[');
					Serial.print(name);
					Serial.println("] endstop released — reapproaching");
					enterReapproach();
				}
				break;

			case HomingPhase::Reapproach:
				if (pressed) {
					finishHomed();
				}
				break;

			case HomingPhase::Done:
				break;
		}
	}
};

AxisHoming homingX{motorX, endstopX, "X"};
AxisHoming homingY{motorY, endstopY, "Y"};
AxisHoming homingZ{motorZ, endstopZ, "Z"};

void setup() {
	Serial.begin(115200);

	pinMode(Config::X::enablePin, OUTPUT);
	digitalWrite(Config::X::enablePin, LOW);

	motorX.begin();
	motorY.begin();
	motorZ.begin();

	Serial.println();
	Serial.println("=== Homing test (seek / backoff / reapproach) ===");
	Serial.print("Home direction: ");
	Serial.println(Config::Z::homeDir == Motor::Direction::Reverse ? "Reverse" : "Forward");
	Serial.print("Seek speed (steps/s): ");
	Serial.println(Config::Z::seekSpeedStepsPerSec);

	homingX.begin();
	homingY.begin();
	homingZ.begin();
}

void loop() {
	motorX.update();
	motorY.update();
	motorZ.update();
	homingX.update();
	homingY.update();
	homingZ.update();
}
