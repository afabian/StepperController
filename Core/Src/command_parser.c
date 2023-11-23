#include "command_parser.h"

// This parses a string of serial data for commands.  Commands are of the form:
// xy=-123.4567890
// They are always followed by a newline or return character.

#include <stdlib.h>
#include <string.h>

enum CommandState {
    RESET = 0,
    EXPECT_LETTER_FIRST = 1,
    EXPECT_LETTER_SECOND = 2,
    EXPECT_EQUALS = 3,
    EXPECT_NUMBERS_OR_EOL = 4
};

enum CommandState state = RESET;
char command[3] = {0};
char value[32] = {0};
int value_len = 0;
char last_command[3] = {0};
double last_value_dbl = 0;
bool last_command_ready = false;

// This system expected to be accessed from two directions:
// 1. The serial port calls command_parse_char for each character it receives.
// 2. The main application polls get_command_received(), and calls get_command() and get_value() whenever a new command is ready.

bool get_command_received() { bool value = last_command_ready; last_command_ready = false; return value; }
char* get_command() { return last_command; }
double get_value() { return last_value_dbl; }

// This is the state machine that does the character-by-character parsing

void command_parse_char(char c) {

	if (state == RESET) {
        bzero(command, sizeof(command));
        bzero(value, sizeof(value));
        value_len = 0;
        state = EXPECT_LETTER_FIRST;
	}

    switch (state) {

    	case RESET:
        case EXPECT_LETTER_FIRST:
            if (c >= 'a' && c <= 'z') {
                command[0] = c;
                state = EXPECT_LETTER_SECOND;
            }
            else {
                state = RESET;
            }
            break;

        case EXPECT_LETTER_SECOND:
            if (c >= 'a' && c <= 'z') {
                command[1] = c;
                state = EXPECT_EQUALS;
            }
            else {
                state = RESET;
            }
            break;

        case EXPECT_EQUALS:
            if (c == '=') {
                state = EXPECT_NUMBERS_OR_EOL;
            }
            else {
                state = RESET;
            }
            break;

        case EXPECT_NUMBERS_OR_EOL:
            if (c == '-' || c == '.' || (c >= '0' && c <= '9')) {
                if (value_len < 31) {
                    value[value_len++] = c;
                }
            }
            else if (c == '\r' || c == '\n') {
                memcpy(last_command, command, sizeof(command));
                char* eptr;
                last_value_dbl = strtod(value, &eptr);
                last_command_ready = true;
                state = RESET;
            }
            else {
                state = RESET;
            }
            break;

    }

}
