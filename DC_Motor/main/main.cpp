#include <stdio.h>
#include <stdint.h>
#include "PinConfig.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static portMUX_TYPE ENCODER_MUX = portMUX_INITIALIZER_UNLOCKED;

volatile int64_t encoder_ticks = 0;

static void encoder_task(void *arg) {
    // Variables for RPM Calculation.
    int64_t last_ticks = 0;
    int64_t current_ticks;
    int64_t delta_ticks;

    uint64_t last_time = esp_timer_get_time(); // Operation Time in Microseconds.
    uint64_t current_time;
    float delta_time_seconds;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(READING_INTERVAL));

        current_time = esp_timer_get_time();

        taskENTER_CRITICAL(&ENCODER_MUX);
        current_ticks = encoder_ticks;
        taskEXIT_CRITICAL(&ENCODER_MUX);

        delta_ticks = current_ticks - last_ticks;
        delta_time_seconds = (current_time - last_time) / 1e6;

        float rpm = (delta_ticks * 60.0f) / (TICKS_PER_ROTATION * delta_time_seconds);

        printf("RPM: %.2f | Delta Ticks: %lld | Delta Time: %4.f\n", rpm, delta_ticks, delta_time_seconds);

        last_ticks = current_ticks;
        last_time = current_time;
    }
}

static void IRAM_ATTR encoder_isr(void *arg) {
    if (gpio_get_level(MOTOR_3_ENCB_PIN) == 1) {
        encoder_ticks = encoder_ticks + 1;
    } else {
        encoder_ticks = encoder_ticks - 1;
    }
}

extern "C" void app_main(void) {
    // Configure All Necessary Pins (Drivers and Encoders).
    configure_pins();
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MOTOR_3_ENCA_PIN, encoder_isr, NULL));

    xTaskCreatePinnedToCore(encoder_task, "encoder_task", 4096, NULL, 5, NULL, 1);
    
    // Set Duty Cicle and Direction:
    uint16_t duty_cycle = 700;
    bool direction = true;  // True for Clockwise, False for Anticlockwise.

    while (true) {
        // Set Motor Direction.
        if (direction) {
            gpio_set_level(MOTOR_3_IN1_PIN, 1);
            gpio_set_level(MOTOR_3_IN2_PIN, 0);
            printf("Direction: Clockwise\n");
        } else {
            gpio_set_level(MOTOR_3_IN1_PIN, 0);
            gpio_set_level(MOTOR_3_IN2_PIN, 1);
            printf("Direction: Anticlockwise\n");
        }

        // Set PWM Duty Cycle to Control Speed.
        ledc_set_duty(MOTOR_PWM_MODE, MOTOR_3_PWM_CHANNEL, duty_cycle);
        ledc_update_duty(MOTOR_PWM_MODE, MOTOR_3_PWM_CHANNEL);

        // Increase the Duty Cycle for Speed Control.
        duty_cycle += 10; // Increase Speed Gradually.
        printf("Duty Cycle: %d\n", duty_cycle);

        if (duty_cycle > MAX_DUTY_CYCLE) {
            // Briefly Disable Motor Between Direction Changes.
            ledc_set_duty(MOTOR_PWM_MODE, MOTOR_3_PWM_CHANNEL, 0);
            ledc_update_duty(MOTOR_PWM_MODE, MOTOR_3_PWM_CHANNEL);

            gpio_set_level(MOTOR_3_IN1_PIN, 0);
            gpio_set_level(MOTOR_3_IN2_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(300)); // Short Delay to Stop.

            // Reset Duty Cycle and Toggle Direction.
            duty_cycle = 700;
            direction = !direction;
            printf("Toggling Direction\n");
        }

        vTaskDelay(pdMS_TO_TICKS(300)); // Delay for Observation.
    }
}