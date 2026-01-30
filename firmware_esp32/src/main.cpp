#include "includes.h"

MotorDC motor1(M1_IN1, M1_IN2, M1_PWM, M1_CANAL_PWM, M1_ENC_A, M1_ENC_B);
MotorDC motor2(M2_IN1, M2_IN2, M2_PWM, M2_CANAL_PWM, M2_ENC_A, M2_ENC_B);
MotorDC motor3(M3_IN1, M3_IN2, M3_PWM, M3_CANAL_PWM, M3_ENC_A, M3_ENC_B);
MotorDC motor4(M4_IN1, M4_IN2, M4_PWM, M4_CANAL_PWM, M4_ENC_A, M4_ENC_B);

void IRAM_ATTR isr_m1() { motor1.lerEncoder(); }
void IRAM_ATTR isr_m2() { motor2.lerEncoder(); }
void IRAM_ATTR isr_m3() { motor3.lerEncoder(); }
void IRAM_ATTR isr_m4() { motor4.lerEncoder(); }

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
    motor1.mover(200);
    motor2.mover(200);
    motor3.mover(200);
    motor4.mover(200);
    
  } else if (tempo_decorrido < 20000) { 
    motor1.mover(-200);
    motor2.mover(-200);
    motor3.mover(-200);
    motor4.mover(-200);
  }
  
  if(tempo_atual - tempo_anterior_print >= 100) {
    printf("Motor 1 RPM: %.2f | Ticks: %ld\n", motor1.calcularRPM(), motor1.getTicks());
    printf("Motor 2 RPM: %.2f | Ticks: %ld\n", motor2.calcularRPM(), motor2.getTicks());
    printf("Motor 3 RPM: %.2f | Ticks: %ld\n", motor3.calcularRPM(), motor3.getTicks());
    printf("Motor 4 RPM: %.2f | Ticks: %ld\n", motor4.calcularRPM(), motor4.getTicks());
    tempo_anterior_print = tempo_atual;
  }
}