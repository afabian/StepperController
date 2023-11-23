#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <stdbool.h>

// This system expected to be accessed from two directions:
// 1. The serial port calls command_parse_char for each character it receives.
// 2. The main application polls get_command_received(), and calls get_command() and get_value() whenever a new command is ready.

bool get_command_received();
char* get_command();
double get_value();

// This parses a string of serial data for commands. Commands are of the form:
// xy=-123.4567890
// They are always followed by a newline or return character.

void command_parse_char(char c);

#endif
