// Standalone X/Y/Z ping-pong test: each axis moves +100 steps, then -100, continuously.

#include <Arduino.h>
#include "Config.h"
#include "Motor.h"

namespace {

static constexpr long kPingPongSteps = 100;

Motor motorX(Config::X::stepPin, Config::X::dirPin);
Motor motorY(Config::Y::stepPin, Config::Y::dirPin);
Motor motorZ(Config::Z::stepPin, Config::Z::dirPin);

struct AxisPingPong {
	Motor& motor;
	const char* name;
	long targetPos = 0;
	long nextDelta = kPingPongSteps;
	bool active = false;

	explicit AxisPingPong(Motor& motorIn, const char* nameIn)
		: motor(motorIn), name(nameIn) {}

	void startLeg(long deltaSteps) {
		targetPos = motor.positionSteps() + deltaSteps;
		active = true;

		motor.enable(true);
		motor.setDirection(deltaSteps > 0 ? Motor::Direction::Reverse : Motor::Direction::Forward);
		motor.setSpeedStepsPerSec(Config::manualSpeedStepsPerSec);
	}

	void begin() {
		nextDelta = kPingPongSteps;
		Serial.print('[');
		Serial.print(name);
		Serial.print("] steps=");
		Serial.println(motor.positionSteps());
		Serial.print('[');
		Serial.print(name);
		Serial.print("] start, delta ");
		Serial.println(nextDelta);
		startLeg(nextDelta);
	}

	void update() {
		if (!active) return;

		const long pos = motor.positionSteps();
		const bool forward = (motor.direction() == Motor::Direction::Forward);
		const bool done = forward ? (pos <= targetPos) : (pos >= targetPos);
		if (!done) return;

		motor.setSpeedStepsPerSec(0);

		Serial.print('[');
		Serial.print(name);
		Serial.print("] steps=");
		Serial.println(pos);

		nextDelta = -nextDelta;

		Serial.print('[');
		Serial.print(name);
		Serial.print("] reverse, delta ");
		Serial.println(nextDelta);

		startLeg(nextDelta);
	}
};

AxisPingPong pingX{motorX, "X"};
AxisPingPong pingY{motorY, "Y"};
AxisPingPong pingZ{motorZ, "Z"};

} // namespace

void setup() {
	Serial.begin(115200);

	pinMode(Config::X::enablePin, OUTPUT);
	digitalWrite(Config::X::enablePin, LOW);

	motorX.begin();
	motorY.begin();
	motorZ.begin();

	pingX.begin();
	pingY.begin();
	pingZ.begin();

	Serial.println();
	Serial.println("=== X/Y/Z ping-pong test ===");
	Serial.print("Each axis: +");
	Serial.print(kPingPongSteps);
	Serial.print(" / -");
	Serial.print(kPingPongSteps);
	Serial.println(" steps, repeating");
	Serial.print("Speed (steps/s): ");
	Serial.println(Config::manualSpeedStepsPerSec);
}

void loop() {
	motorX.update();
	motorY.update();
	motorZ.update();
	pingX.update();
	pingY.update();
	pingZ.update();
}
