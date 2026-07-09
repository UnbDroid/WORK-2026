#ifndef ARM_H
#define ARM_H

#include <ESP32Servo.h>
#include <FastAccelStepper.h>

struct fk_position{
    double fk_x;
    double fk_y;
    double fk_z;
};

struct ik_angle{
    double ik_theta_base;
    double ik_theta_arm;
};

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
        double theta_gripper = PI;
    public:
        Manipulator();
        void init(FastAccelStepper* base, FastAccelStepper* arm, Servo* gripper);
        fk_position forward_kinematics(double fk_theta_base, double fk_theta_arm);
        ik_angle inverse_kinematics(double ik_x, double ik_y, double ik_z);
        void drive_angle(double drive_theta_base, double drive_theta_arm, double drive_theta_gripper);
        void drive_position(double drive_x, double drive_y, double drive_z);

        double get_x_position() {return this->x;}
        double get_y_position() {return this->y;}
        double get_z_position() {return this->z;}
        double get_base_angle() {return this->theta_base;}
        double get_arm_angle() {return this->theta_arm;}
        double get_gripper_angle() {return this->theta_gripper;}
};

#endif // ARM_H