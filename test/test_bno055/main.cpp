// Quick BNO055 accelerometer test: read and print accel (x, y, z) over I2C.
// Wiring: SDA = GPIO 16, SCL = GPIO 17.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "SP_System.h"

static constexpr uint8_t kSdaPin = 16;
static constexpr uint8_t kSclPin = 17;

// Divide accel by g to normalize into units of gravity. When the platform is
// parallel to the ground, the Z accel is ~0 g; tilt makes it grow/shrink ±.
static constexpr float kGravity = 9.8f;

// BNO055 default I2C address is 0x28 (0x29 if ADR is pulled high).
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup() {
	Serial.begin(115200);
	delay(500);

	Wire.begin(kSdaPin, kSclPin);

	Serial.println();
	Serial.println("=== BNO055 accelerometer test ===");

	if (!bno.begin()) {
		Serial.println("BNO055 not found — check wiring (SDA=16, SCL=17) and address (0x28/0x29).");
		while (true) {
			SP::poll();
			delay(100);
		}
	}

	Serial.println("BNO055 ready. Printing normalized accel + Z off-from-parallel:");
}

void loop() {
	SP::poll();

	imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

	// Normalize each axis by gravity -> units of g (~ -1..1).
	const float nx = accel.x() / kGravity;
	const float ny = accel.y() / kGravity;
	const float nz = accel.z() / kGravity;   // ~0 when parallel to the ground

	// Normalized Z is the sine of the tilt, so the angle off from parallel is
	// asin(nz). Clamp first so a slightly-over-1 reading can't blow up asin().
	const float nzClamped = constrain(nz, -1.0f, 1.0f);
	const float tiltDeg = asinf(nzClamped) * 180.0f / PI;

	Serial.print("norm  x=");
	Serial.print(nx, 3);
	Serial.print("  y=");
	Serial.print(ny, 3);
	Serial.print("  z=");
	Serial.print(nz, 3);
	Serial.print("   | off-from-parallel  z(g)=");
	Serial.print(nz, 3);
	Serial.print("  tilt=");
	Serial.print(tiltDeg, 2);
	Serial.println(" deg");

	delay(100);
}
