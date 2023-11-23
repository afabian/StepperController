#ifndef COMMAND_RUNNER_H
#define COMMAND_RUNNER_H

#include <stdbool.h>

typedef struct motionCommand {
	char command[3];
	double value;
} motionCommand;

bool poll_new_command(motionCommand* command);

#endif
