#include <Arduino.h>
#include <ESP32Servo.h>
#include <FastAccelStepper.h>

#include "Config.h"
#include "Arm.h"


Manipulator::Manipulator() {
    this->stepper_base = nullptr;
    this->stepper_arm = nullptr;
    this->servo_gripper = nullptr;
    this->x = 0.0;
    this->y = 0.0;
    this->z = 0.0;
    this->theta_arm = 0.0;
    this->theta_base = 0.0;
    this->theta_gripper = 0.0;
};

void Manipulator::init(FastAccelStepper* base, FastAccelStepper* arm, Servo* gripper){
    stepper_base = base;
    stepper_arm = arm;
    servo_gripper = gripper;
};

void Manipulator::drive_angle(double theta_base, double theta_arm, double theta_gripper){
    this->theta_base = theta_base;
    this->theta_arm = theta_arm;
    this->theta_gripper = theta_gripper;
    
    if (stepper_base) {
        stepper_base->moveTo(theta_base);
    };
    if (stepper_arm) {
        stepper_arm->moveTo(theta_arm);
    };
    if (servo_gripper) {
        servo_gripper->write(theta_gripper);
    };
};

void Manipulator::drive_posi(double x, double y, double z){

};

