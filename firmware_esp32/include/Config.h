#ifndef CONFIG_H
#define CONFIG_H

// Microcontrolador: ESP32_DevKitC_V4.
// Pinos GPIO que Devem Ser Evitados em Geral: 6, 7, 8, 9, 10 e 11 (FLASH).
// Utilizar com Atenção: 0, 2, 5, 12 e 15 (BOOT, PULL-UP/DOWN e SPI) | 1 e 3 (TX/RX).
// Funcionam Somente com Leitura de Dados (INPUT): 34, 35, 36 e 39.

#define PULSOS_POR_VOLTA    1440
#define FREQUENCIA_PWM      5000
#define RESOLUCAO_PWM       8
#define MAX_PWM             255

#define M1_IN1 GPIO_NUM_26      // IN1 Pin Connected to Motor Driver.
#define M1_IN2 GPIO_NUM_27      // IN2 Pin Connected to Motor Driver.
#define M1_PWM GPIO_NUM_13      // Enable Pin for PWM Control.
#define M1_ENC_A GPIO_NUM_16    // Encoder A (Yellow Wire).
#define M1_ENC_B GPIO_NUM_17    // Encoder B Pin (White Wire).
#define M1_CANAL_PWM LEDC_CHANNEL_0
#define M1_KP 1
#define M1_KI 0.25

#define M3_IN1 GPIO_NUM_32
#define M3_IN2 GPIO_NUM_33
#define M3_PWM GPIO_NUM_25
#define M3_ENC_A GPIO_NUM_35
#define M3_ENC_B GPIO_NUM_34
#define M3_CANAL_PWM LEDC_CHANNEL_1
#define M3_KP 1
#define M3_KI 0.25

#define M2_IN1 GPIO_NUM_2
#define M2_IN2 GPIO_NUM_15
#define M2_PWM GPIO_NUM_4
#define M2_ENC_A GPIO_NUM_12
#define M2_ENC_B GPIO_NUM_14
#define M2_CANAL_PWM LEDC_CHANNEL_2
#define M2_KP 1
#define M2_KI 0.25

#define M4_IN1 GPIO_NUM_19
#define M4_IN2 GPIO_NUM_18
#define M4_PWM GPIO_NUM_21
#define M4_ENC_A GPIO_NUM_39
#define M4_ENC_B GPIO_NUM_36
#define M4_CANAL_PWM LEDC_CHANNEL_3
#define M4_KP 1
#define M4_KI 0.25

#endif // CONFIG_H