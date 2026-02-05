#include <stdio.h>
#include <stdint.h>
#include "PinConfig.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h" // For PWM Control.
#include "driver/gpio.h" // For GPIO Control.

extern "C" void app_main(void) {
    // Configure All Necessary Pins (Drivers and Encoders).
    configure_pins();

    // Set Duty Cicle and Direction:
    uint16_t duty_cycle = 700;
    bool direction = true;  // True for Clockwise, False for Anticlockwise.

    while (true) {
        // Set Motor Direction.
        if (direction) {
            gpio_set_level(MOTOR_1_IN1_PIN, 1);
            gpio_set_level(MOTOR_1_IN2_PIN, 0);
            printf("Direction: Clockwise\n");
        } else {
            gpio_set_level(MOTOR_1_IN1_PIN, 0);
            gpio_set_level(MOTOR_1_IN2_PIN, 1);
            printf("Direction: Anticlockwise\n");
        }

        // Set PWM Duty Cycle to Control Speed.
        ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, MOTOR_1_PWM_CHANNEL, duty_cycle));
        ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, MOTOR_1_PWM_CHANNEL));

        // Increase the Duty Cycle for Speed Control.
        duty_cycle += 10; // Increase Speed Gradually.
        printf("Duty Cycle: %d\n", duty_cycle);

        if (duty_cycle > MAX_DUTY_CYCLE) {
            // Briefly Disable Motor Between Direction Changes.
            ledc_set_duty(MOTOR_PWM_MODE, MOTOR_1_PWM_CHANNEL, 0);
            ledc_update_duty(MOTOR_PWM_MODE, MOTOR_1_PWM_CHANNEL);

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