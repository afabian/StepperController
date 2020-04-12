#include "main_real.h"
#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"
#include "iwdg.h"

//////////////////// HELPER FUNCTIONS ///////////////////////

float min(float a, float b) { return a < b ? a : b; }
float max(float a, float b) { return a > b ? a : b; }

void stepper_enable(void) {
	HAL_GPIO_WritePin(STEPPER_ENABLE_GPIO, STEPPER_ENABLE_PIN, GPIO_PIN_RESET);
}

void stepper_disable(void) {
	HAL_GPIO_WritePin(STEPPER_ENABLE_GPIO, STEPPER_ENABLE_PIN, GPIO_PIN_SET);
}

void stepper_set_direction_forward(void) {
	HAL_GPIO_WritePin(STEPPER_DIRECTION_GPIO, STEPPER_DIRECTION_PIN, GPIO_PIN_SET);
}

void stepper_set_direction_reverse(void) {
	HAL_GPIO_WritePin(STEPPER_DIRECTION_GPIO, STEPPER_DIRECTION_PIN, GPIO_PIN_RESET);
}

void led_on(void) {
	HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_RESET);
}

void led_off(void) {
	HAL_GPIO_WritePin(LED_GPIO, LED_PIN, GPIO_PIN_SET);
}

void pulse_generator_start(void) {
	HAL_TIM_OC_Start(&PULSE_TIMER_DEVICE, PULSE_TIMER_CHANNEL);
}

void notify_watchdog(void) {
	HAL_IWDG_Refresh(&hiwdg);
}

void pulse_generator_set_velocity(float velocity_rps) {
	// this should match the setting on the stepper driver
	int motor_steps_per_revolution = 10000;

	// make velocity positive.  reversing is handled elsewhere in the code
	if (velocity_rps < 0) velocity_rps *= -1;

	// this should match the timer's operating frequency (system clock frequency divided by the timer's prescalar plus one)
	int timer_frequency = 2000000;

	// minimum value to emit valid steps.  If the pulse width is set to 10 ticks, then we double this to come up with 20 ticks.
	// values less than this may create erratic / no motion
	int timer_output_min = 20;

	// maximum value is limited by the timer's 16-bit counter
	int timer_output_max = UINT16_MAX;

	// compute the value for the timer's counter that'll give us the correct motor speed
	float steps_per_second = velocity_rps * motor_steps_per_revolution;
	if (steps_per_second == 0) {
		PULSE_TIMER_DEVICE.Instance->ARR = 1;
	}
	else {
		// compute the new timer delay
		float timer_ticks_per_step = timer_frequency / steps_per_second;

		// add a bit of noise to help eliminate quantization at smaller delay values
		float rand_influence = max(0, 1000 - timer_ticks_per_step) / 1000.0;
		timer_ticks_per_step += (((float)rand() / RAND_MAX) - 0.5) * rand_influence;

		// check limits
		timer_ticks_per_step = max(timer_output_min, timer_ticks_per_step);
		timer_ticks_per_step = min(timer_output_max, timer_ticks_per_step);

		// apply to the timer
		PULSE_TIMER_DEVICE.Instance->ARR = timer_ticks_per_step;
	}

}

////////////////////////// MAIN INITIALIZATION FUNCTION ////////////////////

void init_real(void) {
	stepper_enable();
	pulse_generator_start();
}

/////////////////////////// MAIN DATA //////////////////////////////////////

// These are global so that they can be accessed by multiple functions:
// Written to by the serial parser
// Read from by the main loop

float velocity_rps_setpoint = 0;
float acceleration_rps_s = 4.0;

///////////////////////////// MAIN LOOP ////////////////////////////////////

void main_real(void) {
	uint16_t second_counter_ms = 0;
	uint64_t uptime_counter_ms = 0;
	float velocity_rps_current = 0;
	uint8_t rx_buffer = 1;

	while (true) {
		// put the UART in receive mode.  note that the serial port is set to 9600 baud, which puts it at slightly less
		// than 1 byte per loop.  If the baud rate is increased, a better serial rx scheme will have to be developed.
		if (rx_buffer != 0) {
			rx_buffer = 0;
			HAL_UART_Receive_IT(&huart2, &rx_buffer, 1);
		}

		// 1 ms delay forces this loop to run at about 1khz, assuming the rest of it consumes little time
		HAL_Delay(1);
		second_counter_ms = (second_counter_ms + 1) % 1000;
		uptime_counter_ms++;
		float dt = 0.001;

		// see if the UART got anything
		if (rx_buffer != 0) {
			parse_byte(rx_buffer);
		}

		// blink the LED to show that we're alive
		if (second_counter_ms % 50 > 25) {
			led_on();
		}
		else {
			led_off();
		}

		// acceleration control: adjust the actual velocity towards the set velocity at a fixed rate
		if (velocity_rps_current < velocity_rps_setpoint) {
			velocity_rps_current = min(velocity_rps_current + acceleration_rps_s * dt, velocity_rps_setpoint);
		}
		if (velocity_rps_current > velocity_rps_setpoint) {
			velocity_rps_current = max(velocity_rps_current - acceleration_rps_s * dt, velocity_rps_setpoint);
		}

		// set the motor direction
		if (velocity_rps_current >= 0) {
			stepper_set_direction_forward();
		}
		else {
			stepper_set_direction_reverse();
		}

		// set the motor velocity
		pulse_generator_set_velocity(velocity_rps_current);

		// let the watchdog timer know that we're still running
		notify_watchdog();
	}
}

///////////////////////////////// SERIAL PARSER ///////////////////////////

#define MESSAGE_BUFFER_SIZE 24

// This expects two commands on the serial port:
// "v1.234[enter]" - set the velocity target to 1.5 revs/sec
// "a1.234[enter]" - set the acceleration rate for all velocity changes to 5 revs/sec/sec

void parse_byte(uint8_t byte) {
	static uint8_t message_buffer[MESSAGE_BUFFER_SIZE] = {0};
	static int state = 0;
	static int offset = 0;

	switch (state) {
	case 0 /* expecting letter */:
		if (byte == 'v' || byte == 'a') {
			offset = 0;
			message_buffer[offset++] = byte;
			state = 1;
		}
		break;

	case 1 /* expecting number */:

		// if we've got a character that is part of a number, just add it to the message buffer for later processing
		if ((byte >= '0' && byte <= '9') || byte == '.' || byte == '-') {
			message_buffer[offset++] = byte;
			// check to make sure we're not going to overflow the message buffer
			if (offset == MESSAGE_BUFFER_SIZE-1) state = 0;
		}

		// if we got an enter key, process the message buffer
		else if (byte == 13 || byte == 10) {
			message_buffer[offset] = 0;
			float value = atof((char*)&message_buffer[1]);

			// check value for reasonableness, and do the action requested by the user
			if (message_buffer[0] == 'v' && finite(value) && value < 1000 && value > -1000) velocity_rps_setpoint = value;
			if (message_buffer[0] == 'a' && finite(value) && value < 1000 && value > 0) acceleration_rps_s = value;
			state = 0;
		}

		// otherwise we got an unexpected input, so we reset and start over
		else {
			state = 0;
		}
	}
}
