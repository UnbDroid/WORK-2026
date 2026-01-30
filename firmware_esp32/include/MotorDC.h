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
    public:
        MotorDC(uint8_t in1, uint8_t in2, uint8_t pwm, uint8_t canal_pwm, uint8_t enc_a, uint8_t enc_b);
        void init();
        void lerEncoder();
        void mover(int velocidade_pwm);
        float calcularRPM();

        long getTicks() { return this->ticks_contador; }
};

#endif // MOTORDC_H