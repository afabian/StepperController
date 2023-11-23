#include "uptime.h"

// timebase section
// there's a timer that ticks at 1 khz and calls stepper_int()
// that timer also has a counter that counts up microseconds since the last tick
// by counting ticks and adding microseconds, we can get exact microseconds since beginning at any point

volatile unsigned long uptime_us = 0;

TIM_HandleTypeDef* htim = 0;

void uptime_init(TIM_HandleTypeDef* _htim) {
	htim = _htim;
}

unsigned long uptime() {
	unsigned int cnt;
	unsigned long uptime_us_local;

	// this loop is to catch a race condition where cnt rolls around to zero after reading uptime_us but before reading cnt
	do {
		uptime_us_local = uptime_us;
		cnt = htim->Instance->CNT;
	} while (uptime_us_local != uptime_us);

	return uptime_us_local + cnt;
}

void sleep(unsigned int duration) {
	unsigned long stop = uptime() + duration;
	while (uptime() < stop);
}

void uptime_int() {
	uptime_us += 1000;
}
