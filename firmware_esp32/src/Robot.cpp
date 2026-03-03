#include <Arduino.h>
#include "Config.h"
#include "MotorDC.h"
#include "Robot.h"

Robot::Robot(MotorDC& Motor1, MotorDC& Motor2, MotorDC& Motor3, MotorDC& Motor4) : Motor1(Motor1), Motor2(Motor2), Motor3(Motor3), Motor4(Motor4){
    this->Vx = 0.0;
    this->Vy = 0.0;
    this->Omega = 0.0;
}

void Robot::moveRobot(double Vx, double Vy, double Omega) {
    // Conversão de rad/s para m/s:
    float Vx_meters = Vx * WHEEL_RADIUS;
    float Vy_meters = Vy * WHEEL_RADIUS;

    // Cálculo da Cinemática Inversa:
    float V_M1 = Vx_meters + Vy_meters + (DISTANCE_BETWEEN_WHEELS_Y * Omega) - (DISTANCE_BETWEEN_WHEELS_X * Omega);
    float V_M2 = Vx_meters - Vy_meters - (DISTANCE_BETWEEN_WHEELS_Y * Omega) + (DISTANCE_BETWEEN_WHEELS_X * Omega);
    float V_M3 = Vx_meters - Vy_meters + (DISTANCE_BETWEEN_WHEELS_Y * Omega) - (DISTANCE_BETWEEN_WHEELS_X * Omega);
    float V_M4 = Vx_meters + Vy_meters - (DISTANCE_BETWEEN_WHEELS_Y * Omega) + (DISTANCE_BETWEEN_WHEELS_X * Omega);

    // Conversão de m/s para RPM:
    float V_M1_RPM = (V_M1 * 60) / (3.1415 * 2 * WHEEL_RADIUS);
    float V_M2_RPM = (V_M2 * 60) / (3.1415 * 2 * WHEEL_RADIUS);
    float V_M3_RPM = (V_M3 * 60) / (3.1415 * 2 * WHEEL_RADIUS);
    float V_M4_RPM = (V_M4 * 60) / (3.1415 * 2 * WHEEL_RADIUS);

    // Chamada de Função Mover:
    Motor1.mover_rpm(V_M1_RPM);
    Motor2.mover_rpm(V_M2_RPM);
    Motor3.mover_rpm(V_M3_RPM);
    Motor4.mover_rpm(V_M4_RPM);
}