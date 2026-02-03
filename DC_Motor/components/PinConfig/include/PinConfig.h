#ifndef __PIN_CONFIG_H__
#define __PIN_CONFIG_H__

#include "driver/ledc.h" // For PWM Control.
#include "driver/gpio.h" // For GPIO Control.

// Microcontrolador: ESP32_DevKitC_V4.
// Pinos GPIO que Devem Ser Evitados em Geral: 6 a 11 (FLASH) / 1 e 3 (TX|RX).
// Utilizar com Atenção: 0, 2, 12 e 15.
// Funcionam Somente com Leitura de Dados (INPUT): 34, 35, 36 e 39.

// LADO DIREITO - (Motores 1 e 3).
#define MOTOR_1_IN1_PIN GPIO_NUM_26     // IN1 Pin Connected to Motor Driver.
#define MOTOR_1_IN2_PIN GPIO_NUM_27     // IN2 Pin Connected to Motor Driver.
#define MOTOR_1_ENABLE_PIN GPIO_NUM_14  // Enable Pin for PWM Control.
#define MOTOR_1_ENCA_PIN GPIO_NUM_17    // Encoder A (Yellow Wire).
#define MOTOR_1_ENCB_PIN GPIO_NUM_16    // Encoder B Pin (White Wire).

#define MOTOR_3_IN1_PIN GPIO_NUM_32     // IN1 Pin Connected to Motor Driver.
#define MOTOR_3_IN2_PIN GPIO_NUM_33     // IN2 Pin Connected to Motor Driver.
#define MOTOR_3_ENABLE_PIN GPIO_NUM_25  // Enable Pin for PWM Control.
#define MOTOR_3_ENCA_PIN GPIO_NUM_35    // Encoder A (Yellow Wire).
#define MOTOR_3_ENCB_PIN GPIO_NUM_34    // Encoder B Pin (White Wire).

// LADO ESQUERDO - (Motores 2 e 4).
#define MOTOR_2_IN1_PIN GPIO_NUM_2      // IN1 Pin Connected to Motor Driver.
#define MOTOR_2_IN2_PIN GPIO_NUM_15     // IN2 Pin Connected to Motor Driver.
#define MOTOR_2_ENABLE_PIN GPIO_NUM_4   // Enable Pin for PWM Control.
#define MOTOR_2_ENCA_PIN GPIO_NUM_23    // Encoder A (Yellow Wire).
#define MOTOR_2_ENCB_PIN GPIO_NUM_22    // Encoder B Pin (White Wire).

#define MOTOR_4_IN1_PIN GPIO_NUM_18     // IN1 Pin Connected to Motor Driver.
#define MOTOR_4_IN2_PIN GPIO_NUM_19     // IN2 Pin Connected to Motor Driver.
#define MOTOR_4_ENABLE_PIN GPIO_NUM_21  // Enable Pin for PWM Control.
#define MOTOR_4_ENCA_PIN GPIO_NUM_39    // Encoder A (Yellow Wire).
#define MOTOR_4_ENCB_PIN GPIO_NUM_36    // Encoder B Pin (White Wire).

// Configuração dos Encoders:
#define TICKS_PER_ROTATION 1440 // Resolution for Gearbox Motor Shaft Turn.
#define READING_INTERVAL 100 // Measured in Milliseconds.

// Bit Mask (64-bit) para Configuração dos Pinos GPIO dos Encoders.
#define GENERAL_ENCA (1ULL << MOTOR_1_ENCA_PIN) | (1ULL << MOTOR_3_ENCA_PIN) | (1ULL << MOTOR_2_ENCA_PIN) | (1ULL << MOTOR_4_ENCA_PIN)
#define GENERAL_ENCB (1ULL << MOTOR_1_ENCB_PIN) | (1ULL << MOTOR_3_ENCB_PIN) | (1ULL << MOTOR_2_ENCB_PIN) | (1ULL << MOTOR_4_ENCB_PIN)

// Configuração do PWM (LEDC):
#define MOTOR_PWM_FREQ  5000 // Frequency in Hertz for PWM.
#define MOTOR_1_PWM_CHANNEL LEDC_CHANNEL_0
#define MOTOR_3_PWM_CHANNEL LEDC_CHANNEL_1
#define MOTOR_2_PWM_CHANNEL LEDC_CHANNEL_2
#define MOTOR_4_PWM_CHANNEL LEDC_CHANNEL_3
#define MOTOR_PWM_MODE   LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_TIMER  LEDC_TIMER_0
#define MOTOR_PWM_RES    LEDC_TIMER_10_BIT // PWM Resolution (10-bit).
#define MAX_DUTY_CYCLE   1023 // Maximum Duty Cycle for 10-bit Resolution.

// Bit Mask (64-bit) para Configuração dos Pinos GPIO dos Drivers.
#define GENERAL_MOTOR_1 (1ULL << MOTOR_1_IN1_PIN) | (1ULL << MOTOR_1_IN2_PIN) | (1ULL << MOTOR_1_ENABLE_PIN)
#define GENERAL_MOTOR_3 (1ULL << MOTOR_3_IN1_PIN) | (1ULL << MOTOR_3_IN2_PIN) | (1ULL << MOTOR_3_ENABLE_PIN)
#define GENERAL_MOTOR_2 (1ULL << MOTOR_2_IN1_PIN) | (1ULL << MOTOR_2_IN2_PIN) | (1ULL << MOTOR_2_ENABLE_PIN)
#define GENERAL_MOTOR_4 (1ULL << MOTOR_4_IN1_PIN) | (1ULL << MOTOR_4_IN2_PIN) | (1ULL << MOTOR_4_ENABLE_PIN)

// Declaração das Funções de Configuração dos Pinos GPIO:
void configure_pwm(gpio_num_t gpio_num, ledc_channel_t channel);
void configure_pins_input_enca(unsigned long long bit_mask);
void configure_pins_input_encb(unsigned long long bit_mask);
void configure_pins_output_driver(unsigned long long bit_mask);
void configure_pins();

#endif