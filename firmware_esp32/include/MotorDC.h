#ifndef MOTORDC_H
#define MOTORDC_H

#include "includes.h"

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
        unsigned long tempo_anterior;
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
        void mover(float rpm);
        void mover_pwm(float pwm);
        float calcularRPM();
        float updatePID(float rpm_alvo);

        long getTicks() { return this->ticks_contador; }
        long getTicksAnterior() { return this->ticks_anterior; }
        unsigned long getTempoAnterior() { return this->tempo_anterior; }
        float getRPMAtual() { return this->rpm_atual; }
        float getErroAtual() { return this->erro_atual; }
        float getPWMAtual() { return this->pwm_atual; }
};

#endif // MOTORDC_H