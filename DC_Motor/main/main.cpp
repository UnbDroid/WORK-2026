#include <stdio.h>
#include <iostream>
#include "PinConfig.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h" // For PWM Control.
#include "driver/gpio.h" // For GPIO Control.

extern "C" void app_main(void) {
    // Configure GPIO for Motor Direction Control.
    gpio_reset_pin(MOTOR_1_IN1_PIN);
    gpio_set_direction(MOTOR_1_IN1_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(MOTOR_1_IN2_PIN);
    gpio_set_direction(MOTOR_1_IN2_PIN, GPIO_MODE_OUTPUT);
    gpio_reset_pin(MOTOR_1_ENABLE_PIN);
    gpio_set_direction(MOTOR_1_ENABLE_PIN, GPIO_MODE_OUTPUT);

    // Configure Encoder A Pin.
    gpio_config_t enca_config = {
        .pin_bit_mask   = (1ULL << MOTOR_1_ENCA_PIN),
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_ENABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_POSEDGE
    };
    gpio_config(&enca_config);

    // Configure Encoder B Pin.
    gpio_config_t encb_config = {
        .pin_bit_mask   = (1ULL << MOTOR_1_ENCB_PIN),
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_ENABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };
    gpio_config(&encb_config);

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
        .gpio_num       = MOTOR_1_ENABLE_PIN,
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
            gpio_set_level(MOTOR_1_IN1_PIN, 1);
            gpio_set_level(MOTOR_1_IN2_PIN, 0);
            printf("Direction: Clockwise\n");
        } else {
            gpio_set_level(MOTOR_1_IN1_PIN, 0);
            gpio_set_level(MOTOR_1_IN2_PIN, 1);
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

            gpio_set_level(MOTOR_1_IN1_PIN, 0);
            gpio_set_level(MOTOR_1_IN2_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(200)); // Short Delay to Stop.

            // Reset Duty Cycle and Toggle Direction.
            duty_cycle = 700;
            direction = !direction;
            printf("Toggling Direction\n");
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Delay for Observation.
    }
}