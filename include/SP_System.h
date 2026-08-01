#pragma once

// SP_System — software reset over serial.
//
// Press 'r' (or 'R') in the serial monitor to reboot the ESP32. No Enter needed:
// `pio device monitor` sends each keystroke immediately, so a single 'r' fires it.
//
// Usage: include this header and call SP::poll() once near the TOP of loop():
//
//   #include <SP_System.h>
//   void loop() {
//     SP::poll();
//     ... your code ...
//   }
//
// It only reads the byte when it is 'r'/'R' (via peek), so any other serial input
// (jog commands like "X100", pause 'd', etc.) is left in the buffer for the rest of
// your loop to parse. Works identically in the production firmware and every test.

#include <Arduino.h>

namespace SP {

// Reboot immediately if the next serial byte is the reset trigger.
inline void poll() {
	if (Serial.available() <= 0) return;

	const int next = Serial.peek();
	if (next != 'r' && next != 'R') return; // not for us — leave it for the app parser

	Serial.read(); // consume the trigger byte
	Serial.println();
	Serial.println("[SP] Software reset requested — restarting...");
	Serial.flush(); // ensure the message is sent before we reboot
	ESP.restart();
}

} // namespace SP
