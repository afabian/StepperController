#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "main.h"
#include <stdbool.h>

void stepper_enable();

void stepper_disable();

void stepper_direction(bool forward);

void stepper_step();

void stepper_step_direction(bool forward);

#endif
