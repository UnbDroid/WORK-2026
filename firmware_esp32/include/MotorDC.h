#ifndef MOTORDC_H
#define MOTORDC_H

#include <Arduino.h>
#include "Config.h"
#include "driver/ledc.h"

class MotorDC {
    private:
        uint8_t in1;
        uint8_t in2;
        uint8_t pwm;
        uint8_t enc_a;
        uint8_t enc_b;
        uint8_t canal_pwm;
        volatile long ticks_contador;
        long ticks_anterior;
        unsigned long tempo_anterior_rpm;
        unsigned long tempo_anterior_pi;
        bool direction; // (false = Straight) | (true = Backwards). 
        float rpm_atual;
        float rpm_alvo;
        float pwm_atual;
        float erro_atual;
        float erro_acumulado;
        const float Kp;
        const float Ki;
    public:
        MotorDC(uint8_t in1, uint8_t in2, uint8_t pwm, uint8_t canal_pwm, uint8_t enc_a, uint8_t enc_b, float Kp, float Ki);
        void init();
        void lerEncoder();
        void mover_rpm(float rpm);
        void mover_pwm(uint32_t pwm, bool direction);
        float calcularRPM();
        float updatePID(float rpm_alvo);

        long getTicks() { return this->ticks_contador; }
        long getTicksAnterior() { return this->ticks_anterior; }
        float getRPMAtual() { return this->rpm_atual; }
        float getErroAtual() { return this->erro_atual; }
        float getPWMAtual() { return this->pwm_atual; }
        bool getDirection() { return this->direction; }
};

#endif // MOTORDC_H