#pragma once

#include "Motor.h"

class Search {
public:
	struct Config {
		long minSteps = 0;
		long maxSteps = 0;
		float speedStepsPerSec = 25.0f;
	};

	explicit Search(const Config& cfg) : cfg_(cfg) {}

	void start(Motor& motor);
	void update(Motor& motor);
	void pause(Motor& motor);
	void resume(Motor& motor);
	bool isPaused() const { return paused_; }

private:
	Config cfg_;
	bool paused_ = false;
};
