#include <Arduino.h>
#include "Config.h"
#include "MotorDC.h"
#include "Robot.h"

constexpr double YAW_PARAMETER = (DISTANCE_BETWEEN_WHEELS_X + DISTANCE_BETWEEN_WHEELS_Y);

Robot::Robot(MotorDC& Motor1, MotorDC& Motor2, MotorDC& Motor3, MotorDC& Motor4) : Motor1(Motor1), Motor2(Motor2), Motor3(Motor3), Motor4(Motor4){
    this->Vx = 0.0;
    this->Vy = 0.0;
    this->Omega = 0.0;
}

void Robot::moveRobot(double Vx, double Vy, double Omega) {
    // Cálculo de Cinemática Inversa:
    double V_M1 = Vx + Vy + YAW_PARAMETER * Omega;
    double V_M2 = Vx - Vy - YAW_PARAMETER * Omega;
    double V_M3 = Vx - Vy + YAW_PARAMETER * Omega;
    double V_M4 = Vx + Vy - YAW_PARAMETER * Omega;

    // Conversão: Metros por Segundo => RPM:
    double V_M1_RPM = (V_M1 * 60.f) / (PI * 2.f * WHEEL_RADIUS);
    double V_M2_RPM = (V_M2 * 60.f) / (PI * 2.f * WHEEL_RADIUS);
    double V_M3_RPM = (V_M3 * 60.f) / (PI * 2.f * WHEEL_RADIUS);
    double V_M4_RPM = (V_M4 * 60.f) / (PI * 2.f * WHEEL_RADIUS);

    // Chamada de Função Mover:
    Motor1.mover_rpm(V_M1_RPM);
    Motor2.mover_rpm(V_M2_RPM);
    Motor3.mover_rpm(V_M3_RPM);
    Motor4.mover_rpm(V_M4_RPM);
}