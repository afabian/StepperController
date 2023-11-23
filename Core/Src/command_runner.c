#include "command_runner.h"
#include "command_parser.h"

#include <string.h>

// this wraps the real command parser (command_parser.c) by adding an error-checking mechanism.
// since there are no checksums or other error-detection mechanisms in this protocol, we implement
// a crude one by requiring each command to be sent twice.  The command is only executed when
// received the second time with identical contents.
//
// this should be called frequently by the main loop.
//
// when it returns true, a command is available and has been stored in the motionCommand pointer parameter

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
