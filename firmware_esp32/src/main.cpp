#include "includes.h"

MotorDC motor1(M1_IN1, M1_IN2, M1_PWM, M1_CANAL_PWM, M1_ENC_A, M1_ENC_B, M1_KP, M1_KI);
MotorDC motor2(M2_IN1, M2_IN2, M2_PWM, M2_CANAL_PWM, M2_ENC_A, M2_ENC_B, M2_KP, M2_KI);
MotorDC motor3(M3_IN1, M3_IN2, M3_PWM, M3_CANAL_PWM, M3_ENC_A, M3_ENC_B, M3_KP, M3_KI);
MotorDC motor4(M4_IN1, M4_IN2, M4_PWM, M4_CANAL_PWM, M4_ENC_A, M4_ENC_B, M4_KP, M4_KI);

void IRAM_ATTR isr_m1() { motor1.lerEncoder(); }
void IRAM_ATTR isr_m2() { motor2.lerEncoder(); }
void IRAM_ATTR isr_m3() { motor3.lerEncoder(); }
void IRAM_ATTR isr_m4() { motor4.lerEncoder(); }

int pwm_teste = 0;          
unsigned long tempo_anterior_passo = 0;
const int PASSO_TEMPO = 1500; 
const int PASSO_PWM = 5;      
const int PWM_MAXIMO_TESTE = 120; 

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

  Serial.println("=== INICIANDO TESTE DE ZONA MORTA ===");
  Serial.println("O PWM vai aumentar gradualmente. Observe quando o RPM sair do zero.");
  delay(2000);
}

void loop() {
  unsigned long tempo_atual = millis();

  if (tempo_atual - tempo_anterior_passo >= PASSO_TEMPO) {
    tempo_anterior_passo = tempo_atual;

    motor1.mover_pwm(pwm_teste);
    motor2.mover_pwm(pwm_teste);
    motor3.mover_pwm(pwm_teste);
    motor4.mover_pwm(pwm_teste);

    printf("\n--- TESTANDO PWM: %d ---\n", pwm_teste);

    printf("M1: %.2f RPM %s\n", motor1.getRPMAtual(), (motor1.getRPMAtual() > 1.0) ? "[MOVENDO!]" : ".");
    printf("M2: %.2f RPM %s\n", motor2.getRPMAtual(), (motor2.getRPMAtual() > 1.0) ? "[MOVENDO!]" : ".");
    printf("M3: %.2f RPM %s\n", motor3.getRPMAtual(), (motor3.getRPMAtual() > 1.0) ? "[MOVENDO!]" : ".");
    printf("M4: %.2f RPM %s\n", motor4.getRPMAtual(), (motor4.getRPMAtual() > 1.0) ? "[MOVENDO!]" : ".");

    pwm_teste += PASSO_PWM;

    if (pwm_teste > PWM_MAXIMO_TESTE) {
      motor1.mover_pwm(0);
      motor2.mover_pwm(0);
      motor3.mover_pwm(0);
      motor4.mover_pwm(0);
      Serial.println("\n--- FIM DO CICLO. REINICIANDO EM 5 SEGUNDOS ---");
      pwm_teste = 0;
      delay(5000); 
    }
  }
}