#include <Arduino.h>
#include <ESP32Servo.h>
#include <FastAccelStepper.h>

#include "Config.h"
#include "Arm.h"

constexpr double BASE_STEPS_PER_RAD = (STEPS_PER_REV_BASE * MICROSTEPPING_MULT) / (2.0 * PI);

constexpr double ARM_STEPS_PER_RAD = (STEPS_PER_REV_ARM * MICROSTEPPING_MULT) / (2.0 * PI);

inline int32_t baseRadToSteps(double rad)
{
    return lround(rad * BASE_STEPS_PER_RAD);
}

inline int32_t armRadToSteps(double rad)
{
    return lround(rad * ARM_STEPS_PER_RAD);
}

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
        stepper_base->moveTo(baseRadToSteps(theta_base));
    };
    if (stepper_arm) {
        stepper_arm->moveTo(armRadToSteps(theta_arm));
    };
    if (servo_gripper) {
        servo_gripper->write(theta_gripper * 180.0 / PI);
    };
};

void Manipulator::drive_posi(double x, double y, double z){

};

