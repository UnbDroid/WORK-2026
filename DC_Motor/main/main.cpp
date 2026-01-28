#include <stdio.h>
#include <iostream>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"  // For PWM Control.
#include "driver/gpio.h"  // For GPIO Control.

#define MOTOR_IN1_PIN   GPIO_NUM_27  // IN1 Pin Connected to Motor Driver.
#define MOTOR_IN2_PIN   GPIO_NUM_26  // IN2 Pin Connected to Motor Driver.
#define MOTOR_ENABLE_PIN GPIO_NUM_32 // Enable Pin for PWM Control.

#define MOTOR_PWM_FREQ  5000                  // Frequency in Hz for PWM.
#define MOTOR_PWM_CHANNEL LEDC_CHANNEL_0
#define MOTOR_PWM_MODE   LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_TIMER  LEDC_TIMER_0
#define MOTOR_PWM_RES    LEDC_TIMER_10_BIT    // PWM Resolution (10-bit).
#define MAX_DUTY_CYCLE   1023                 // Maximum Duty Cycle for 10-bit Resolution.

extern "C" void app_main(void) {
    // Configure GPIO for Motor Direction Control.
    gpio_reset_pin(MOTOR_IN1_PIN);
    gpio_set_direction(MOTOR_IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(MOTOR_IN2_PIN);
    gpio_set_direction(MOTOR_IN2_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(MOTOR_ENABLE_PIN);
    gpio_set_direction(MOTOR_ENABLE_PIN, GPIO_MODE_OUTPUT);

    // Configure PWM Timer.
    ledc_timer_config_t pwm_timer = {
        .speed_mode       = MOTOR_PWM_MODE,
        .duty_resolution  = MOTOR_PWM_RES,
        .timer_num        = MOTOR_PWM_TIMER,
        .freq_hz          = MOTOR_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    // Configure PWM Channel.
    ledc_channel_config_t pwm_channel = {
        .gpio_num       = MOTOR_ENABLE_PIN,
        .speed_mode     = MOTOR_PWM_MODE,
        .channel        = MOTOR_PWM_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = MOTOR_PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&pwm_channel);

    int duty_cycle = 700;
    int direction = 1;  // 1 for Clockwise, 0 for Anticlockwise.

    while (true) {
        // Set Motor Direction.
        if (direction == 1) {
            gpio_set_level(MOTOR_IN1_PIN, 1);
            gpio_set_level(MOTOR_IN2_PIN, 0);
            printf("Direction: Clockwise\n");
        } else {
            gpio_set_level(MOTOR_IN1_PIN, 0);
            gpio_set_level(MOTOR_IN2_PIN, 1);
            printf("Direction: Anticlockwise\n");
        }

        // Set PWM Duty Cycle to Control Speed.
        ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL, duty_cycle);
        ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL);

        // Increase the Duty Cycle for Speed Control.
        duty_cycle += 10; // Increase Speed Gradually.
        printf("Duty Cycle: %d\n", duty_cycle);

        if (duty_cycle > MAX_DUTY_CYCLE) {
            // Briefly Disable Motor Between Direction Changes.
            ledc_set_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL, 0);
            ledc_update_duty(MOTOR_PWM_MODE, MOTOR_PWM_CHANNEL);

            gpio_set_level(MOTOR_IN1_PIN, 0);
            gpio_set_level(MOTOR_IN2_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(200)); // Short Delay to Stop.

            // Reset Duty Cycle and Toggle Direction.
            duty_cycle = 700;
            direction = !direction;
            printf("Toggling Direction\n");
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Delay for Observation.
    }
}