#include <stdint.h>
#include "PinConfig.h"
#include "esp_log.h"

void configure_pwm_timer(void){
    // Configure PWM Timer.
    ledc_timer_config_t pwm_timer = {
        .speed_mode       = MOTOR_PWM_MODE,
        .duty_resolution  = MOTOR_PWM_RES,
        .timer_num        = MOTOR_PWM_TIMER,
        .freq_hz          = MOTOR_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK,
        .deconfigure      = false
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));
};

void configure_pwm_channel(gpio_num_t gpio_num, ledc_channel_t channel) {
    // Configure PWM Channel.
    ledc_channel_config_t pwm_channel = {
        .gpio_num       = gpio_num,
        .speed_mode     = MOTOR_PWM_MODE,
        .channel        = channel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = MOTOR_PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0,
        .sleep_mode     = LEDC_SLEEP_MODE_NO_ALIVE_ALLOW_PD,
        .flags          = {.output_invert = false}
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
}

void configure_pins_input_enca(uint64_t bit_mask){
    // Configure Encoder A Pin.
    gpio_config_t enca_config = {
        .pin_bit_mask   = bit_mask,
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_ENABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_POSEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&enca_config));
};

void configure_pins_input_encb(uint64_t bit_mask){
    // Configure Encoder B Pin.
    gpio_config_t encb_config = {
        .pin_bit_mask   = bit_mask,
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_ENABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&encb_config));
};

void configure_pins_output_driver(uint64_t bit_mask){
    // Configure Motor Direction Control Pin.
    gpio_config_t driver_config = {
        .pin_bit_mask   = bit_mask,
        .mode           = GPIO_MODE_OUTPUT,
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&driver_config));
};

void configure_pins(){
    // PWM Timer Config.
    configure_pwm_timer();

    // PWM Channel Config.
    configure_pwm_channel(MOTOR_1_ENABLE_PIN, MOTOR_1_PWM_CHANNEL);
    configure_pwm_channel(MOTOR_3_ENABLE_PIN, MOTOR_3_PWM_CHANNEL);
    configure_pwm_channel(MOTOR_2_ENABLE_PIN, MOTOR_2_PWM_CHANNEL);
    configure_pwm_channel(MOTOR_4_ENABLE_PIN, MOTOR_4_PWM_CHANNEL);

    // Encoders Pin Config.
    configure_pins_input_enca(GENERAL_ENCA);
    configure_pins_input_encb(GENERAL_ENCB);

    // Drivers Pin Config.
    configure_pins_output_driver(GENERAL_MOTOR_1);
    configure_pins_output_driver(GENERAL_MOTOR_3);
    configure_pins_output_driver(GENERAL_MOTOR_2);
    configure_pins_output_driver(GENERAL_MOTOR_4);

    ESP_LOGI("PIN_CFG", "Pinos Configurados");
};