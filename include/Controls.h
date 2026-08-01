#pragma once

// =============================================================================
// Controls.h  —  Serial control reference (documentation only, no code)
// =============================================================================
//
// A single place listing every command you can type in the serial monitor,
// which firmware it applies to, when it's allowed, and what it does.
//
// Monitor:  pio device monitor -b 115200   (sends each keystroke immediately)
//   - Single-key controls (r, c, d) fire on the keypress  — NO Enter needed.
//   - Jog commands (X10, Y-10, 200, ...) are read as a line — press Enter.
//
//
// -----------------------------------------------------------------------------
// GLOBAL  —  available in EVERY firmware (production + all tests)
// -----------------------------------------------------------------------------
// Provided by SP_System.h (SP::poll() at the top of each loop()).
//
//   r / R    Software reset. Reboots the ESP32 (same as the EN button).
//            Works ANY time — during homing, mid-move, while idle.
//
//
// -----------------------------------------------------------------------------
// PRODUCTION FIRMWARE  —  src/main.cpp   (pio run -t upload)
// -----------------------------------------------------------------------------
// Homing runs automatically on boot; commands below work once homed.
//
//   d / D    Stop / pause. Halts motion. If not yet homed, aborts homing.
//            If a search sweep is running, pauses it. Fires immediately.
//   c / C    Resume the search sweep. Only when the sweep is paused. Immediate.
//   r / R    Software reset (see GLOBAL). Immediate.
//
//   Jog commands (press Enter; allowed once homed, and when search is paused):
//     200 or -200      Move Z by signed steps.
//     X10  / X-10      Move X by signed steps.
//     Y10  / Y-10      Move Y by signed steps.
//
//
// -----------------------------------------------------------------------------
// TEST: test_zero_motors   (pio test -f test_zero_motors --without-testing)
// -----------------------------------------------------------------------------
// Homes X/Y in lockstep on boot, zeros both, then enters manual jog.
//
//   r / R    Software reset (see GLOBAL). Immediate, incl. during homing.
//
//   Jog commands (press Enter; only after "Homing complete"):
//     X100 / X-110     Move X by signed steps.
//     Y20  / Y-20      Move Y by signed steps.
//     X 10 Y 20        Move X and Y together in one line.
//   After each command, running per-axis totals print ([Total X] / [Total Y]).
//   Note: positive delta -> Forward, negative -> Reverse (direction flipped).
//
//
// -----------------------------------------------------------------------------
// TEST: test_movement   (pio test -f test_movement --without-testing)
// -----------------------------------------------------------------------------
// Manual X/Y/Z jog. No homing.
//
//   r / R    Software reset (see GLOBAL). Immediate.
//
//   Jog commands (press Enter):
//     X100 / X-100     Move X by signed steps.
//     Y50  / Y-50      Move Y by signed steps.
//     Z-3  / Z3        Move Z by signed steps.
//     X 100 Y 100 Z 100  Move all three axes at once.
//   Note: positive delta -> Reverse, negative -> Forward.
//
//
// -----------------------------------------------------------------------------
// TEST: test_home   (pio test -f test_home --without-testing)
// -----------------------------------------------------------------------------
// Each axis runs seek / backoff / reapproach homing on boot. No jog commands.
//
//   r / R    Software reset (see GLOBAL). Immediate — handy to re-run homing.
//
//
// -----------------------------------------------------------------------------
// TEST: test_endstop   (pio test -f test_endstop --without-testing)
// -----------------------------------------------------------------------------
// Prints endstop state changes (PRESSED / RELEASED). No jog commands.
//
//   r / R    Software reset (see GLOBAL). Immediate.
//
//
// -----------------------------------------------------------------------------
// TEST: test_xy_motors   (pio test -f test_xy_motors --without-testing)
// -----------------------------------------------------------------------------
// X/Y/Z ping-pong (+100 / -100 steps) forever. Starts on boot. No jog commands.
//
//   r / R    Software reset (see GLOBAL). Immediate.
//
// =============================================================================
