#include "command_runner.h"
#include "command_parser.h"

#include <string.h>

bool poll_new_command(motionCommand* command) {

	static char command_a[3] = {0};
	static double value_a = 0;

	static char command_b[3] = {0};
	static double value_b = 0;

    // as an error-catching method we want commands to be repeated twice with identical content before executing them.

    if (get_command_received()) {

        memcpy(command_a, command_b, sizeof(command_a));
        value_a = value_b;

        memcpy(command_b, get_command(), sizeof(command_b));
        value_b = get_value();

        if (command_a[0] == command_b[0] && command_a[1] == command_b[1] && value_a == value_b) {
        	command->command[0] = command_a[0];
        	command->command[1] = command_a[1];
        	command->value = value_a;
        	return true;
        }

    }

    return false;

}
