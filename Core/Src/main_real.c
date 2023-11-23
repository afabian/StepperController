#include "main_real.h"
#include "stepper.h"
#include "uptime.h"
#include "command_runner.h"
#include "motion.h"

#define DT_US 100

int last_idle_time = 0;
unsigned long next_start_time = 0;
int actual_position_steps = 0;
int immediate_position_steps = 0;
bool last_enabled = false;

motionCommand new_command = {0};

// This needs to be compiled with some level of optimization, or it's on the edge of not making timing.

void main_real() {

	stepper_enable();

	while (1) {

		// run at a constant loop rate defined by DT_US
		next_start_time += DT_US;
		last_idle_time = (long)next_start_time - (long)uptime();
		while (uptime() < next_start_time) ;

		// read any new commands from the serial port
		if(poll_new_command(&new_command)) {
			motion_command(&new_command);
		}

		// enable/disable the motor if necessary
		bool enabled = motion_get_enabled();
		if (enabled && !last_enabled) {
			stepper_enable();
		}
		if (!enabled && last_enabled) {
			stepper_disable();
		}
		last_enabled = enabled;

		// update the motion plan since some time has passed, and see what step we should be on
		immediate_position_steps = motion_get_position_target_steps();

		// send one step to the stepper motor if necessary
		// this can take up to about 10 us
		int position_error_steps = immediate_position_steps - actual_position_steps;
		if (position_error_steps > 0) {
			stepper_step_direction(true);
			actual_position_steps++;
		}
		if (position_error_steps < 0) {
			stepper_step_direction(false);
			actual_position_steps--;
		}

	}
}
