#include "includes.h"

MotorDC::MotorDC(uint8_t in1, uint8_t in2, uint8_t pwm, uint8_t canal_pwm, uint8_t enc_a, uint8_t enc_b, float Kp, float Ki)
    : in1(in1), in2(in2), pwm(pwm), canal_pwm(canal_pwm), enc_a(enc_a), enc_b(enc_b), Kp(Kp), Ki(Ki)
{
    this->ticks_contador = 0; 
    this->ticks_anterior = 0;
    this->tempo_anterior = 0;
    this->erro_acumulado = 0;
}

void MotorDC::init() {
    pinMode(this->in1, OUTPUT);
    pinMode(this->in2, OUTPUT);
    
    ledcSetup(this->canal_pwm, FREQUENCIA_PWM, RESOLUCAO_PWM); 
    ledcAttachPin(this->pwm, this->canal_pwm);

    pinMode(this->enc_a, INPUT);
    pinMode(this->enc_b, INPUT);
}

void MotorDC::lerEncoder() {
    int estado_b = digitalRead(this->enc_b);
    
    if (estado_b > 0) {
        this->ticks_contador++;
    } else {
        this->ticks_contador--;
    }
}

void MotorDC::mover(float rpm) {
    this->rpm_alvo = rpm;
    int pwm_controlado = this->updatePID(this->rpm_alvo);

    if (pwm_controlado > 255) pwm_controlado = 255;
    if (pwm_controlado < -255) pwm_controlado = -255;

    if (pwm_controlado > 0) {
        digitalWrite(this->in1, HIGH);
        digitalWrite(this->in2, LOW);
    } else if (pwm_controlado < 0) {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, HIGH);
        pwm_controlado = abs(pwm_controlado);
    } else {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, LOW);
    }

    ledcWrite(this->canal_pwm, pwm_controlado);
}

void MotorDC::mover_pwm(float pwm) {
    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;

    if (pwm > 0) {
        digitalWrite(this->in1, HIGH);
        digitalWrite(this->in2, LOW);
    } else if (pwm < 0) {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, HIGH);
        pwm = abs(pwm);
    } else {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, LOW);
    }

    ledcWrite(this->canal_pwm, pwm);
}

float MotorDC::calcularRPM() {
    unsigned long tempo_atual = millis();
    unsigned long delta_tempo = tempo_atual - this->tempo_anterior;

    if(delta_tempo < 10) {
        return this->rpm_atual;
    } 

    noInterrupts();
    long ticks_atual = this->ticks_contador;
    interrupts();

    long delta_ticks = ticks_atual - this->ticks_anterior;

    float rpm = ((float)delta_ticks / 1440) * (60000.0 / delta_tempo);
    this->rpm_atual = rpm;

    this->ticks_anterior = ticks_atual;
    this->tempo_anterior = tempo_atual;

    return this->rpm_atual;
}

float MotorDC::updatePID(float rpm_alvo) {
    if(rpm_alvo > 0 && this->rpm_alvo < 0 || rpm_alvo < 0 && this->rpm_alvo > 0) {
        this->erro_acumulado = 0;
    }

    this->rpm_alvo = rpm_alvo;
    this->erro_atual = this->rpm_alvo - this->calcularRPM();

    this->erro_acumulado += this->erro_atual;

    float pwm_proporcional = this->Kp * this->erro_atual;
    float pwm_integral = this->Ki * this->erro_acumulado;

    if (pwm_integral > 255) {
        pwm_integral = 255;
        this->erro_acumulado = 255 / this->Ki; 
    } else if (pwm_integral < -255) {
        pwm_integral = -255;
        this->erro_acumulado = -255 / this->Ki;
    }

    float pwm_controlado = pwm_proporcional + pwm_integral;

    if (pwm_controlado > 255) pwm_controlado = 255;
    if (pwm_controlado < -255) pwm_controlado = -255;

    this->pwm_atual = pwm_controlado;
    return pwm_controlado; 
}