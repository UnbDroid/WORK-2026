#include "PinConfig.h"
#include <stdio.h>
#include <iostream>

void configure_pwm(gpio_num_t gpio_num, ledc_channel_t channel){
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
        .gpio_num       = gpio_num,
        .speed_mode     = MOTOR_PWM_MODE,
        .channel        = channel,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = MOTOR_PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&pwm_channel);
}

void configure_pins_input_enca(unsigned long long bit_mask){
    // Configure Encoder A Pin.
    gpio_config_t enca_config = {
        .pin_bit_mask   = bit_mask,
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_ENABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_POSEDGE
    };
    gpio_config(&enca_config);
};

void configure_pins_input_encb(unsigned long long bit_mask){
    // Configure Encoder B Pin.
    gpio_config_t encb_config = {
        .pin_bit_mask   = bit_mask,
        .mode           = GPIO_MODE_INPUT,
        .pull_up_en     = GPIO_PULLUP_ENABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };
    gpio_config(&encb_config);
};

void configure_pins_output_driver(unsigned long long bit_mask){
    // Configure Motor Direction Control Pin.
    gpio_config_t driver_config = {
        .pin_bit_mask   = bit_mask,
        .mode           = GPIO_MODE_OUTPUT,
        .pull_up_en     = GPIO_PULLUP_DISABLE,
        .pull_down_en   = GPIO_PULLDOWN_DISABLE,
        .intr_type      = GPIO_INTR_DISABLE
    };
    gpio_config(&driver_config);
};

void configure_pins(){
    // PWM Config.
    configure_pwm(MOTOR_1_ENABLE_PIN, MOTOR_1_PWM_CHANNEL);
    configure_pwm(MOTOR_3_ENABLE_PIN, MOTOR_3_PWM_CHANNEL);
    configure_pwm(MOTOR_2_ENABLE_PIN, MOTOR_2_PWM_CHANNEL);
    configure_pwm(MOTOR_4_ENABLE_PIN, MOTOR_4_PWM_CHANNEL);

    // Encoders Pin Config.
    configure_pins_input_enca(GENERAL_ENCA);
    configure_pins_input_encb(GENERAL_ENCB);

    // Drivers Pin Config.
    configure_pins_output_driver(GENERAL_MOTOR_1);
    configure_pins_output_driver(GENERAL_MOTOR_3);
    configure_pins_output_driver(GENERAL_MOTOR_2);
    configure_pins_output_driver(GENERAL_MOTOR_4);

    std::cout << "Pinos Configurados" << std::endl;
};