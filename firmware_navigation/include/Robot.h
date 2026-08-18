#ifndef ROBOT_H
#define ROBOT_H

#include "MotorDC.h"

class Robot {
    private:
        MotorDC& Motor1;
        MotorDC& Motor2;
        MotorDC& Motor3;
        MotorDC& Motor4;
        double Vx;
        double Vy;
        double Omega;
    public:
        Robot(MotorDC& Motor1, MotorDC& Motor2, MotorDC& Motor3, MotorDC& Motor4);
        void moveRobot(double Vx, double Vy, double Omega);
};

#endif // ROBOT_H