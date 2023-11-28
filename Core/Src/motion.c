#include "motion.h"
#include "uptime.h"
#include <math.h>

// default values for velocity limit, acceleration limit, and steps per revolution.
// these can be overridden by commands when running.
float vl = 90;
float al = 10;
int steps_per_rev = 25000;

// initial and target (final) positions, velocities, and times
double pf = 0;
double vf = 0;
unsigned long t0 = 0;
double v0 = 0;
double p0 = 0;

// if we're current in position target mode (if not then we're in velocity target mode)
bool target_p_mode = true;

// special mode where we decel to a stop before switching to position mode.
// this is necessary because our position mode math can't handle cases where v0 isn't zero.
bool stop_needed = false;

// if the motor is enabled or not.  this should match the default in the main loop.
bool enabled = true;

// the commanded values for p and v that we will calculate
float v_cmd = 0;
float p_cmd = 0;

int sign(double value) {
	return value > 0 ? 1 : -1;
}

// this accepts commands from command_parser / command_runner.  commands are two letters and a number,
// so those get decoded into actual function calls here.
//
// supported commands:
//
// en=0 - disable motor power
// en=1 - enable motor power
// mv=X - set max velocity to X
// ma=X - set max acceleration to X
// sr=X - set the steps-per-revolution* value to X
// tp=X - command a target position of X
// tv=X - command a target velocity of X
//
// * steps-per-revolution is the number of steps required to rotate the final device (after any gearing)
//   by one revolution.  This can be the product of three things:
//   stepper motor's steps-per-revolution, which is typically 200
//   stepper driver's microstepping setting
//   any mechanical advantage obtained through gearing or belts

void motion_command(motionCommand* command) {

	// Enable / Disable motor
	if (command->command[0] == 'e' && command->command[1] == 'n') {
		enabled = command->value != 0;
	}

	// Configure Max Velocity (deg/sec)
	if (command->command[0] == 'm' && command->command[1] == 'v') {
		vl = command->value;
	}

	// Configure Max Acceleration (deg/sec^2)
	if (command->command[0] == 'm' && command->command[1] == 'a') {
		al = command->value;
	}

	// Configure Steps per Revolution
	if (command->command[0] == 's' && command->command[1] == 'r') {
		steps_per_rev = command->value;
	}

	// Position Command (deg)
	if (command->command[0] == 't' && command->command[1] == 'p') {
		pf = command->value;
		vf = 0;
		target_p_mode = true;
		t0 = uptime();
		p0 = p_cmd;
		v0 = v_cmd;
		stop_needed = true;
	}

	// Velocity Command (deg/sec)
	if (command->command[0] == 't' && command->command[1] == 'v') {
		vf = command->value;
		pf = 0;
		target_p_mode = false;
		t0 = uptime();
		p0 = p_cmd;
		v0 = v_cmd;
		stop_needed = false;
	}

}

// main motion control command
// this outputs a target step position for the current moment in time.
// its up to the parent code to issue steps to the motor to get it to this position.

int motion_get_position_target_steps() {

	if (target_p_mode) /* position mode */ {
		return motion_get_position_target_steps_position_mode();
	}

	else /* velocity mode */ {
		return motion_get_position_target_steps_velocity_mode();
	}

}

// velocity mode

int motion_get_position_target_steps_velocity_mode() {

	// set our velocity target - either the commanded velocity target if we're in that mode,
	// or zero if we're in position mode but need to do a stop first
	float v_tgt;
	if (stop_needed) {
		v_tgt = 0;
		if (v_cmd == 0) {
			stop_needed = false;
			t0 = uptime();
			p0 = p_cmd;
			v0 = 0;
		}
	}
	else {
		v_tgt = vf;
	}

	unsigned long now = uptime();
	unsigned long tc = fabs((v_tgt - v0) * 1000000) / al;
	float a = sign(vf - v0) * al;
	float t = (now - t0) / 1000000.0f;

	if (now > t0 + tc) /* holding at target velocity */ {
		float tcus = tc / 1000000.0f;
		float dt = (now - tc - t0) / 1000000.0f;
		v_cmd = v_tgt;
		p_cmd = p0 + v0 * tcus + 0.5 * a * tcus * tcus + v_tgt * dt;
	}

	else /* accelerating to target velocity */ {
		v_cmd = v0 + a * t;
		p_cmd = p0 + v0 * t + 0.5 * a * t * t;
	}

	// translation the target position from degrees to steps
	return p_cmd / 360.0f * steps_per_rev;
}

// position mode

int motion_get_position_target_steps_position_mode() {

	// this assumes that v0 and vf are both zero!
	// an improved version could be made with support for arbitrary initial (and final?!) velocities.
	// this would let us switch from velocity mode to position mode without stopping first.

	unsigned long now = uptime();

	float t01 = vl / al;
	int psign = sign(pf - p0);
	float a = psign * al;
	float v = psign * vl;
	float p01 = 0.5f * a * t01 * t01;
	float p12 = pf - p0 - 2 * p01;

	// special case for if we're doing small movements that will never reach max velocity and have
	// just accel and decel phases
	if (fabs(p01) > 0.5f * fabs(pf - p0)) {
		p01 = 0.5f * (pf - p0);
		p12 = 0;
		t01 = sqrtf(2 * p01 / a);
	}

	float t12 = fabs(p12) / vl;

	unsigned long t1 = t0 + t01 * 1000000;
	unsigned long t2 = t1 + t12 * 1000000;
	unsigned long t3 = t2 + t01 * 1000000;

	float v1 = a * t01;

	float t = 0;

	if (now > t3) /* done; resting at target position */ {
		v_cmd = 0;
		p_cmd = pf;
	}
	else if (now > t2) /* deceleration phase */ {
		t = (now - t2) * 0.000001f;
		v_cmd = v1 - a * t;
		p_cmd = p0 + p01 + p12 + v1 * t - 0.5 * a * t * t;
	}
	else if (now > t1) /* constant-velocity phase */ {
		t = (now - t1) * 0.000001f;
		v_cmd = v;
		p_cmd = p0 + p01 + v * t;
	}
	else /* acceleration phase */ {
		t = (now - t0) * 0.000001f;
		v_cmd = a * t;
		p_cmd = p0 + 0.5f * a * t * t;
	}

	// translation the target position from degrees to steps
	return p_cmd / 360.0f * steps_per_rev;
}

bool motion_get_enabled() {
	return enabled;
}
