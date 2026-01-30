#include "includes.h"

MotorDC::MotorDC(uint8_t in1, uint8_t in2, uint8_t pwm, uint8_t canal_pwm, uint8_t enc_a, uint8_t enc_b)
    : in1(in1), in2(in2), pwm(pwm), canal_pwm(canal_pwm), enc_a(enc_a), enc_b(enc_b) 
{
    this->ticks_contador = 0; 
    this->ticks_anterior = 0;
    this->tempo_anterior = 0;
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

void MotorDC::mover(int velocidade_pwm) {

    if (velocidade_pwm > 255) velocidade_pwm = 255;
    if (velocidade_pwm < -255) velocidade_pwm = -255;

    if (velocidade_pwm > 0) {
        digitalWrite(this->in1, HIGH);
        digitalWrite(this->in2, LOW);
    } else if (velocidade_pwm < 0) {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, HIGH);
        velocidade_pwm = abs(velocidade_pwm);
    } else {
        digitalWrite(this->in1, LOW);
        digitalWrite(this->in2, LOW);
    }

    ledcWrite(this->canal_pwm, velocidade_pwm);
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

    this->rpm_atual = (delta_ticks / PULSOS_POR_VOLTA) * (60000.0 / delta_tempo);

    this->ticks_anterior = ticks_atual;
    this->tempo_anterior = tempo_atual;

    return this->rpm_atual;
}