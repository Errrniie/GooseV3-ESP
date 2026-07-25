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
static constexpr uint8_t endstopPin = 22;
} // namespace X

namespace Y {
static constexpr uint8_t stepPin = 25;
static constexpr uint8_t dirPin = 26;
static constexpr uint8_t enablePin = 27;
static constexpr uint8_t endstopPin = 23;
} // namespace Y

namespace Z {
static constexpr uint8_t stepPin = 32;
static constexpr uint8_t dirPin = 33;
static constexpr uint8_t enablePin = 27;
static constexpr uint8_t endstopPin = 13;

// Homing profile for this axis (wired into Homing::Config in main.cpp).
static constexpr Motor::Direction homeDir = Motor::Direction::Reverse;
static constexpr float seekSpeedStepsPerSec = 40.0f;
static constexpr float backoffSpeedStepsPerSec = 20.0f;
// Final homing touch that sets the zero: crawl so debounce/loop latency costs a
// tiny fraction of a step (was 50 -> ~1 step of scatter = ~1.1" at 3 ft).
static constexpr float reapproachSpeedStepsPerSec = 10.0f;
// Extra steps to back off *past* switch release before reapproaching, so the final
// touch always crawls in over the same distance regardless of where boot/reset left it.
static constexpr long backoffMarginSteps = 6;
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
