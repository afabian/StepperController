#ifndef INC_UPTIME_H_
#define INC_UPTIME_H_

#include "main.h"
#include <stdbool.h>

void uptime_int();

void uptime_init(TIM_HandleTypeDef* _htim);

unsigned long uptime();

void sleep(unsigned int duration);

#endif
