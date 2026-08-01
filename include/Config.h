#pragma once

#include <Arduino.h>
#include "Motor.h"

// Application configuration: pins, mechanics, non-homing motion, search sweep, and app flags.
namespace Config {

// -------- Driver microstepping (single source of truth) --------
// MUST match the closed-loop driver's DIP/software setting.
//   16 => 3200 pulses/rev on a 1.8° (200 full-step) motor.
// Every step COUNT and step-RATE below is written as (baseline * microstepping),
// where the baseline is the value that was correct at microstepping = 1. Changing
// this one number rescales all motion so physical distances and speeds are
// preserved while resolution gets 'microstepping' times finer.
static constexpr long microstepping = 16;

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
// Speeds are (baseline full-steps/sec * microstepping): physical speed unchanged,
// but the final crawl now resolves to 1 microstep instead of 1 full step.
static constexpr Motor::Direction homeDir = Motor::Direction::Reverse;
static constexpr float seekSpeedStepsPerSec = 40.0f * microstepping;
static constexpr float backoffSpeedStepsPerSec = 20.0f * microstepping;

// Final homing touch that sets the zero: crawl so debounce/loop latency costs a
// tiny fraction of a step (baseline 10 full-steps/s; scatter now ~1.1"/microstepping at 3 ft).
static constexpr float reapproachSpeedStepsPerSec = 10.0f * microstepping;

// Extra steps to back off *past* switch release before reapproaching, so the final
// touch always crawls in over the same distance regardless of where boot/reset left it.
static constexpr long backoffMarginSteps = 6 * microstepping;

static constexpr float travelSpeedStepsPerSec = 70.0f * microstepping;
static constexpr long finalPositionSteps = -350 * microstepping;
static constexpr uint32_t finalMoveTimeoutMs = 60000;
} // namespace Z

// -------- Endstop (shared wiring assumptions) --------
static constexpr uint16_t endstopDebounceMs = 10;
// NO switch to GND + INPUT_PULLUP: idle = HIGH, pressed = LOW → true.
// Set false if pressed reads HIGH (e.g. NC or active-high wiring).
static constexpr bool endstopPressedIsPinLow = true;

// -------- Mechanics / units (single source of truth for motor motion) --------
// fullStepsPerRev is the MOTOR's native resolution (1.8 deg stepper = 200/rev) --
// a hardware fact, NOT what we command in. Microstepping multiplies it:
//
//   effective stepsPerRev = 200 * 16 = 3200  -> 0.1125 deg per commanded pulse.
//
// We move in microsteps, so a full revolution is 3200 pulses (no lost detail).
// Anything that moves a motor by an angle derives its step count from these --
// use stepsPerRev / degreesPerStep / stepsForDegrees(), never the raw 200.
static constexpr long fullStepsPerRev = 200;                            // motor: 1.8 deg full step
static constexpr long stepsPerRev = fullStepsPerRev * microstepping;    // EFFECTIVE: 3200 at 16x
static constexpr float degreesPerStep = 360.0f / static_cast<float>(stepsPerRev);  // 0.1125 at 16x
static constexpr float stepsPerDegree = static_cast<float>(stepsPerRev) / 360.0f;  // 8.889 at 16x
static constexpr long stepsForDegrees(float deg) { return (long)(deg * stepsPerDegree + (deg < 0 ? -0.5f : 0.5f)); }

// -------- Manual motion (post-homing jog: Z, X, Y) --------
static constexpr float manualSpeedStepsPerSec = 70.0f * microstepping;

// -------- Search sweep --------
static constexpr long searchMinSteps = 210 * microstepping;
static constexpr long searchMaxSteps = 500 * microstepping;
static constexpr float searchSpeedStepsPerSec = 70.0f * microstepping;

// -------- Neutral / working home --------
// After the endstop home (0,0), the head moves here and treats it as the resting
// "home"/neutral point. Absolute step-pulses at the current microstepping,
// measured/calibrated by jogging (the [Total X]/[Total Y] readout).
namespace Neutral {
static constexpr long x = 655;
static constexpr long y = 630;
} // namespace Neutral

// -------- App behavior --------
static constexpr bool afterHomingManualTrackingTest = true;

} // namespace Config
