#pragma once

#include <Arduino.h>
#include "Motor.h"

// Application configuration: pins, mechanics, non-homing motion, search sweep, and app flags.
namespace Config {

// -------- Pins (step, dir, enable, endstop per axis) --------
namespace X {
static constexpr uint8_t stepPin = 18;
static constexpr uint8_t dirPin = 19;
static constexpr uint8_t enablePin = 27;
static constexpr uint8_t endstopPin = 34;
} // namespace X

namespace Y {
static constexpr uint8_t stepPin = 25;
static constexpr uint8_t dirPin = 26;
static constexpr uint8_t enablePin = 27;
static constexpr uint8_t endstopPin = 35;
} // namespace Y

namespace Z {
static constexpr uint8_t stepPin = 32;
static constexpr uint8_t dirPin = 33;
static constexpr uint8_t enablePin = 27;
static constexpr uint8_t endstopPin = 13;

// Homing profile for this axis (wired into Homing::Config in main.cpp).
static constexpr Motor::Direction homeDir = Motor::Direction::Reverse;
static constexpr float seekSpeedStepsPerSec = 70.0f;
static constexpr float backoffSpeedStepsPerSec = 16.0f;
static constexpr float reapproachSpeedStepsPerSec = 12.0f;
static constexpr float travelSpeedStepsPerSec = 70.0f;
static constexpr long finalPositionSteps = -350;
static constexpr uint32_t finalMoveTimeoutMs = 60000;
} // namespace Z

// -------- Endstop (shared wiring assumptions) --------
static constexpr uint16_t endstopDebounceMs = 10;
// NO switch to GND + INPUT_PULLUP: idle = HIGH, pressed = LOW → true.
// Set false if pressed reads HIGH (e.g. NC or active-high wiring).
static constexpr bool endstopPressedIsPinLow = true;

// -------- Mechanics / units --------
static constexpr long fullStepsPerRev = 720;
static constexpr long microstepping = 1;
static constexpr long maxSteps = fullStepsPerRev * microstepping;

static constexpr float maxAngleDeg = 1296.0f;
static constexpr float degreesPerStep = maxAngleDeg / static_cast<float>(maxSteps);

// -------- Manual motion (post-homing jog: Z, X, Y) --------
static constexpr float manualSpeedStepsPerSec = 70.0f;

// -------- Search sweep --------
static constexpr long searchMinSteps = 210;
static constexpr long searchMaxSteps = 500;
static constexpr float searchSpeedStepsPerSec = 70.0f;

// -------- App behavior --------
static constexpr bool afterHomingManualTrackingTest = true;

} // namespace Config
