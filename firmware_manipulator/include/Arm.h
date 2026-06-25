#ifndef ARM_H
#define ARM_H

#include <ESP32Servo.h>
#include <FastAccelStepper.h>

class Manipulator {
    private:
        FastAccelStepper* stepper_base;
        FastAccelStepper* stepper_arm;
        Servo* servo_gripper;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double theta_base = 0.0;
        double theta_arm = 0.0;
        double theta_gripper = 0.0;
    public:
        Manipulator();
        void init(FastAccelStepper* base, FastAccelStepper* arm, Servo* gripper);
        void drive_angle(double theta_base, double theta_arm, double theta_gripper);
        void drive_posi(double x, double y, double z);
};

#endif // ARM_H