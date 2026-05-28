#pragma once

#include <Arduino.h>

class Endstop {
public:
	explicit Endstop(uint8_t pin, uint16_t debounceMs = 0, bool pressedMeansPinLow = true)
			: pin_(pin), debounceMs_(debounceMs), pressedMeansPinLow_(pressedMeansPinLow) {}

	void begin() {
		pinMode(pin_, INPUT_PULLUP);
		rawState_ = readRawPressed_();
		stableState_ = rawState_;
		lastRawChangeMs_ = millis();
	}

	void update() {
		if (debounceMs_ == 0) {
			stableState_ = readRawPressed_();
			return;
		}

		const bool raw = readRawPressed_();
		const uint32_t now = millis();

		if (raw != rawState_) {
			rawState_ = raw;
			lastRawChangeMs_ = now;
		}

		if (stableState_ != rawState_ && (now - lastRawChangeMs_) >= debounceMs_) {
			stableState_ = rawState_;
		}
	}

	bool isPressed() const { return stableState_; }

private:
	bool readRawPressed_() const {
		const bool pinLow = (digitalRead(pin_) == LOW);
		return pressedMeansPinLow_ ? pinLow : !pinLow;
	}

	uint8_t pin_;
	uint16_t debounceMs_ = 0;
	bool pressedMeansPinLow_ = true;
	bool rawState_ = false;
	bool stableState_ = false;
	uint32_t lastRawChangeMs_ = 0;
};
