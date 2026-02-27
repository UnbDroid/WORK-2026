#include <Arduino.h>
#include "Config.h"
#include "MotorDC.h"
#include "driver/ledc.h"

MotorDC motor1(M1_IN1, M1_IN2, M1_PWM, M1_CANAL_PWM, M1_ENC_A, M1_ENC_B, M1_KP, M1_KI);
MotorDC motor2(M2_IN1, M2_IN2, M2_PWM, M2_CANAL_PWM, M2_ENC_A, M2_ENC_B, M2_KP, M2_KI);
MotorDC motor3(M3_IN1, M3_IN2, M3_PWM, M3_CANAL_PWM, M3_ENC_A, M3_ENC_B, M3_KP, M3_KI);
MotorDC motor4(M4_IN1, M4_IN2, M4_PWM, M4_CANAL_PWM, M4_ENC_A, M4_ENC_B, M4_KP, M4_KI);

void IRAM_ATTR isr_m1() { motor1.lerEncoder(); }
void IRAM_ATTR isr_m2() { motor2.lerEncoder(); }
void IRAM_ATTR isr_m3() { motor3.lerEncoder(); }
void IRAM_ATTR isr_m4() { motor4.lerEncoder(); }

void moveRobot (float Vx, float Vy, float Omega) {
  // unsigned long tempo_anterior = 0;
  // unsigned long tempo_atual = millis();

  // Conversão de rad/s para m/s:
  float Vx_meters = Vx * WHEEL_RADIUS;
  float Vy_meters = Vy * WHEEL_RADIUS;

  // Cálculo da Cinemática Inversa:
  float V_M1 = Vx_meters + Vy_meters - (DISTANCE_BETWEEN_WHEELS_Y * Omega) + (DISTANCE_BETWEEN_WHEELS_X * Omega);
  float V_M2 = Vx_meters - Vy_meters + (DISTANCE_BETWEEN_WHEELS_Y * Omega) - (DISTANCE_BETWEEN_WHEELS_X * Omega);
  float V_M3 = Vx_meters - Vy_meters + (DISTANCE_BETWEEN_WHEELS_Y * Omega) - (DISTANCE_BETWEEN_WHEELS_X * Omega);
  float V_M4 = Vx_meters + Vy_meters - (DISTANCE_BETWEEN_WHEELS_Y * Omega) + (DISTANCE_BETWEEN_WHEELS_X * Omega);

  // if (tempo_atual - tempo_anterior >= 500) {
  // Serial.printf("\nV_M1: %.2f", V_M1);
  // Serial.printf("\nV_M2: %.2f", V_M2);
  // Serial.printf("\nV_M3: %.2f", V_M3);
  // Serial.printf("\nV_M4: %.2f", V_M4);
  // tempo_anterior = tempo_atual;
  // }

  // Conversão de m/s para RPM:
  float V_M1_RPM = (V_M1 * 60) / (3.1415 * 2 * WHEEL_RADIUS);
  float V_M2_RPM = (V_M2 * 60) / (3.1415 * 2 * WHEEL_RADIUS);
  float V_M3_RPM = (V_M3 * 60) / (3.1415 * 2 * WHEEL_RADIUS);
  float V_M4_RPM = (V_M4 * 60) / (3.1415 * 2 * WHEEL_RADIUS);

  // if (tempo_atual - tempo_anterior >= 500) {
  // Serial.printf("\nV_M1: %.2f", V_M1_RPM);
  // Serial.printf("\nV_M2: %.2f", V_M2_RPM);
  // Serial.printf("\nV_M3: %.2f", V_M3_RPM);
  // Serial.printf("\nV_M4: %.2f", V_M4_RPM);
  // tempo_anterior = tempo_atual;
  // }

  // Chamada de Função Mover:
  motor1.mover_rpm(V_M1_RPM);
  motor2.mover_rpm(V_M2_RPM);
  motor3.mover_rpm(V_M3_RPM);
  motor4.mover_rpm(V_M4_RPM);
}

unsigned long tempo_anterior_print = 0;
unsigned long tempo_decorrido = 0;
unsigned long tempo_inicio = 0;

void setup() {
  Serial.begin(115200);

  motor1.init();
  motor2.init();
  motor3.init();
  motor4.init();

  attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr_m1, RISING);
  attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isr_m2, RISING);
  attachInterrupt(digitalPinToInterrupt(M3_ENC_A), isr_m3, RISING);
  attachInterrupt(digitalPinToInterrupt(M4_ENC_A), isr_m4, RISING);

  tempo_inicio = millis();
}

void loop() {
  unsigned long tempo_atual = millis();
  tempo_decorrido = tempo_atual - tempo_inicio;

if (tempo_decorrido < 10000) { 
    moveRobot(0.0f, 5.0f, 0.0f);
    //  motor1.mover_rpm(50);
    //  motor2.mover_rpm(50);
    //  motor3.mover_rpm(50);
    //  motor4.mover_rpm(50);
    
  } else if (tempo_decorrido < 20000) { 
    moveRobot(0.0f, -5.0f, 0.0f);
    // motor1.mover_rpm(-50);
    // motor2.mover_rpm(-50);
    // motor3.mover_rpm(-50);
    // motor4.mover_rpm(-50);

  } else {
    moveRobot(0, 0, 0);

  }
  
  if(tempo_atual - tempo_anterior_print >= 500) {
    Serial.printf("Motor 1 | RPM: %.2f | Erro: %.2f | PWM: %.2f\n", motor1.getRPMAtual(), motor1.getErroAtual(), motor1.getPWMAtual());
    Serial.printf("Motor 2 | RPM: %.2f | Erro: %.2f | PWM: %.2f\n", motor2.getRPMAtual(), motor2.getErroAtual(), motor2.getPWMAtual());
    Serial.printf("Motor 3 | RPM: %.2f | Erro: %.2f | PWM: %.2f\n", motor3.getRPMAtual(), motor3.getErroAtual(), motor3.getPWMAtual());
    Serial.printf("Motor 4 | RPM: %.2f | Erro: %.2f | PWM: %.2f\n", motor4.getRPMAtual(), motor4.getErroAtual(), motor4.getPWMAtual());
    tempo_anterior_print = tempo_atual;
  }
}