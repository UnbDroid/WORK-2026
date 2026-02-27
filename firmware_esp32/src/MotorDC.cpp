#include <Arduino.h>
#include "Config.h"
#include "MotorDC.h"
#include "driver/ledc.h"

MotorDC::MotorDC(uint8_t in1, uint8_t in2, uint8_t pwm, uint8_t canal_pwm, uint8_t enc_a, uint8_t enc_b, float Kp, float Ki)
    : in1(in1), in2(in2), pwm(pwm), canal_pwm(canal_pwm), enc_a(enc_a), enc_b(enc_b), Kp(Kp), Ki(Ki)
{
    this->ticks_contador = 0; 
    this->ticks_anterior = 0;
    this->tempo_anterior_rpm = 0;
    this->tempo_anterior_pi = 0;
    this->direction = false;
    this->rpm_atual = 0.0;
    this->rpm_anterior = 0.0f;
    this->rpm_alvo = 0.0f;
    this->pwm_atual = 0.0f;
    this->erro_atual = 0.0;
    this->erro_acumulado = 0.0;
}

void MotorDC::init() {
    pinMode(this->in1, OUTPUT);
    pinMode(this->in2, OUTPUT);
    
    ledcSetup(this->canal_pwm, PWM_FREQUENCY, PWM_RESOLUTION); 
    ledcAttachPin(this->pwm, this->canal_pwm);

    pinMode(this->enc_a, INPUT_PULLUP);
    pinMode(this->enc_b, INPUT_PULLUP);
}

void MotorDC::lerEncoder() {
    int estado_b = digitalRead(this->enc_b);
    
    if (estado_b > 0) {
        this->ticks_contador++;
    } else {
        this->ticks_contador--;
    }
}

void MotorDC::mover_rpm(float rpm) {
    this->rpm_alvo = rpm;
    int pwm_controlado = (int) round(this->updatePID(this->rpm_alvo));

    // Tratamento de Direção.
    this->direction = (rpm_alvo < 0);

    pwm_controlado = abs(pwm_controlado);
    if (pwm_controlado > MAX_PWM) pwm_controlado = MAX_PWM;

    if (pwm_controlado > 0 && direction == false) {
        digitalWrite(this->in1, HIGH);
        digitalWrite(this->in2, LOW);
    } else if (pwm_controlado > 0 && direction == true) {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, HIGH);
    } else {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, LOW);
    }

    ledcWrite(this->canal_pwm, pwm_controlado);
}

void MotorDC::mover_pwm(uint32_t pwm, bool direction) {
    if (pwm > MAX_PWM) pwm = MAX_PWM;

    if (pwm > 0 && direction == false) {
        digitalWrite(this->in1, HIGH);
        digitalWrite(this->in2, LOW);
    } else if (pwm > 0 && direction == true) {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, HIGH);
    } else {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, LOW);
    }

    ledcWrite(this->canal_pwm, pwm);
}

float MotorDC::calcularRPM() {
    const int RESOLUTION = 10; // Intervalo em Milisegundos Para Leitura.
    unsigned long tempo_atual = millis();
    unsigned long delta_tempo = tempo_atual - this->tempo_anterior_rpm;

    if(delta_tempo < RESOLUTION) {
        return this->rpm_atual;
    } 

    noInterrupts();
    long ticks_atual = this->ticks_contador;
    interrupts();

    long delta_ticks = ticks_atual - this->ticks_anterior;

    float rpm = ((float)delta_ticks / TICKS_PER_REVOLUTION) * (60000.0f / delta_tempo);
    this->rpm_atual = rpm;

    this->ticks_anterior = ticks_atual;
    this->tempo_anterior_rpm = tempo_atual;

    return this->rpm_atual;
}

double MotorDC::updatePID(float rpm_alvo) {
    const float MAX_PWM_FLOAT = 255.0f;
    const float MIN_PWM_FLOAT = -255.0f;
    const double MAX_RPM = 100.0;
    const float DEADZONE = 115.0f;
    const float ZERO_BAND = 1.0f;

    // Verificação de Mudança de Direção (Reseta o Erro Acumulado Linearmente).
    if (rpm_alvo * this->rpm_anterior < 0) { this->erro_acumulado *= 0.2; }
    this->rpm_anterior = rpm_alvo;
    
    this->rpm_alvo = rpm_alvo;
    this->erro_atual = this->rpm_alvo - this->calcularRPM();

    // Reseta o Acúmulo de Erro e Envia (PWM = 0) Caso Robô Esteja em Zona de Parada.
    if (fabs(rpm_alvo) < ZERO_BAND)
    {
        this->erro_acumulado *= 0.2;
        this->pwm_atual = 0.0f;
        return 0.0;
    }

    // Feedforward - Determinação do PWM Aproximado para Atingir o Alvo.
    double initial_pwm = ((MAX_PWM_FLOAT - DEADZONE) / MAX_RPM) * rpm_alvo;

    // Tratamento de Zona Morta de PWM dos Motores.
    if (fabs(rpm_alvo) > ZERO_BAND) {
        initial_pwm += (rpm_alvo > 0 ? DEADZONE : -DEADZONE);
    } else {
        initial_pwm = 0;
    }

    double pwm_proporcional = this->Kp * this->erro_atual;

    unsigned long tempo_atual = millis();
    double dt = (tempo_atual - this->tempo_anterior_pi) * 0.001;
    this->tempo_anterior_pi = tempo_atual;

    if (dt < 0.001) dt = 0.001;

    double integral = this->erro_acumulado + erro_atual * dt;
    double pwm_integral = this->Ki * integral;

    double pwm_controlado = pwm_proporcional + pwm_integral;

    double final_pwm = initial_pwm + pwm_controlado;

    // Tratamento de Anti-Windup e Saturação (Acúmulo de Erro Somente em Banda de Operação).
    if (final_pwm > MAX_PWM_FLOAT) {
        final_pwm = MAX_PWM_FLOAT;
    } else if (final_pwm < MIN_PWM_FLOAT) {
        final_pwm = MIN_PWM_FLOAT;
    } else {
        erro_acumulado = integral;
    }
    
    // Envio de Módulo do PWM Controlado.
    this->pwm_atual = fabs(final_pwm);
    return final_pwm; 
}