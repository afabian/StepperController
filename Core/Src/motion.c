#include "motion.h"
#include "uptime.h"
#include <math.h>

float vl = 90;
float al = 10;
int steps_per_rev = 25000;

double pf = 0;
double vf = 0;
bool target_p_mode = true;
unsigned long t0 = 0;
double v0 = 0;
double p0 = 0;

bool enabled = true;

float last_v = 0;
float last_p = 0;

int sign(double value) {
	return value > 0 ? 1 : -1;
}

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
		p0 = last_p;
		v0 = last_v;
	}

	// Velocity Command (deg/sec)
	if (command->command[0] == 't' && command->command[1] == 'v') {
		vf = command->value;
		pf = 0;
		target_p_mode = false;
		t0 = uptime();
		p0 = last_p;
		v0 = last_v;
	}

}

int motion_get_position_target_steps() {

	if (target_p_mode) /* position mode */ {
		return motion_get_position_target_steps_position_mode();
	}

	else /* velocity mode */ {
		return motion_get_position_target_steps_velocity_mode();
	}

}

int motion_get_position_target_steps_velocity_mode() {
	// this mode is subdivided into two phases: either accelerating to the target velocity, or holding steady at the target velocity
	unsigned long now = uptime();
	unsigned long tc = fabs((vf - v0) * 1000000) / al;
	float a = sign(vf - v0) * al;
	float t = (now - t0) / 1000000.0f;
	if (now > t0 + tc) {
		float tcus = tc / 1000000.0f;
		float dt = (now - tc - t0) / 1000000.0f;
		last_v = vf;
		last_p = p0 + v0 * tcus + 0.5 * a * tcus * tcus + vf * dt;
	}
	else {
		last_v = v0 + a * t;
		last_p = p0 + v0 * t + 0.5 * a * t * t;
	}

	return last_p / 360.0f * steps_per_rev;
}

int motion_get_position_target_steps_position_mode() {

	// this assumes that v0 and vf are both zero!

	unsigned long now = uptime();

	float t01 = vl / al;
	int psign = sign(pf - p0);
	float a = psign * al;
	float v = psign * vl;
	float p01 = 0.5f * a * t01 * t01;
	float p12 = pf - p0 - 2 * p01;

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

	if (now > t3) {
		last_v = 0;
		last_p = pf;
	}
	else if (now > t2) {
		t = (now - t2) * 0.000001f;
		last_v = v1 - a * t;
		last_p = p0 + p01 + p12 + v1 * t - 0.5 * a * t * t;
	}
	else if (now > t1) {
		t = (now - t1) * 0.000001f;
		last_v = v;
		last_p = p0 + p01 + v * t;
	}
	else {
		t = (now - t0) * 0.000001f;
		last_v = a * t;
		last_p = p0 + 0.5f * a * t * t;
	}

	return last_p / 360.0f * steps_per_rev;
}

bool motion_get_enabled() {
	return enabled;
}
