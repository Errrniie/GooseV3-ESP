#pragma once

#include <Arduino.h>

enum class AppState : uint8_t {
	Homing,
	Searching,
	Tracking
};
