#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#define STEPPER_ENABLE_GPIO GPIOB
#define STEPPER_ENABLE_PIN  GPIO_PIN_4

#define STEPPER_DIRECTION_GPIO GPIOB
#define STEPPER_DIRECTION_PIN  GPIO_PIN_3

#define LED_GPIO GPIOC
#define LED_PIN  GPIO_PIN_13

#define PULSE_TIMER_DEVICE htim2
#define PULSE_TIMER_CHANNEL TIM_CHANNEL_1

float min(float a, float b);
float max(float a, float b);

void stepper_enable(void);
void stepper_disable(void);
void stepper_set_direction_forward(void);
void stepper_set_direction_reverse(void);
void led_on(void);
void led_off(void);
void pulse_generator_start(void);
void pulse_generator_set_velocity(float velocity_rps);
void notify_watchdog(void);

void init_real(void);
void main_real(void);

void parse_byte(uint8_t byte);
