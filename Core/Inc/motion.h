#ifndef MOTION_H
#define MOTION_H

#include "command_runner.h"
#include <stdbool.h>

void motion_command(motionCommand* command);
int motion_get_position_target_steps();
int motion_get_position_target_steps_position_mode();
int motion_get_position_target_steps_velocity_mode();
bool motion_get_enabled();
int sign(double value);

#endif
