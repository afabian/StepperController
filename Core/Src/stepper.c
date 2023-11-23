#include "stepper.h"
#include "uptime.h"

// low-level stepper control
// general individual forward and backwards steps,
// or turn the motor power on and off

void stepper_enable() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	sleep(100);
}

void stepper_disable() {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	sleep(100);
}

void stepper_direction(bool forward) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
	// direction signal must lead pulse by at least 5 us
	sleep(5);
}

bool last_step_forward = false;

void stepper_step_direction(bool forward) {
	if (forward != last_step_forward) {
		stepper_direction(forward);
		last_step_forward = forward;
	}
	stepper_step();
}

void stepper_step() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	// pulse must be at least 2.5 us
	sleep(3);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
}
