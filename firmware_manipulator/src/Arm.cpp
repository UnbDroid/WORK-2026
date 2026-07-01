#include <Arduino.h>
#include <cmath>
#include <ESP32Servo.h>
#include <FastAccelStepper.h>

#include "Config.h"
#include "Arm.h"

constexpr double BASE_STEPS_PER_RAD = (STEPS_PER_REV_BASE * MICROSTEPPING_MULT) / (2.0 * PI);

constexpr double ARM_STEPS_PER_RAD = (STEPS_PER_REV_ARM * MICROSTEPPING_MULT) / (2.0 * PI);

inline int32_t base_rad_to_steps(double rad)
{
    return lround(rad * BASE_STEPS_PER_RAD);
}

inline int32_t arm_rad_to_steps(double rad)
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

    if (stepper_base) {
        stepper_base->setDirectionPin(DIR_M1_PIN);
        stepper_base->setEnablePin(ENABLE_M1_PIN);
        stepper_base->setAutoEnable(true);

        stepper_base->setSpeedInHz(1.0 * BASE_STEPS_PER_RAD);
        stepper_base->setAcceleration(0.4 * BASE_STEPS_PER_RAD);
    }

    if (stepper_arm) {
        stepper_arm->setDirectionPin(DIR_M2_PIN);
        stepper_arm->setEnablePin(ENABLE_M2_PIN);
        stepper_arm->setAutoEnable(false);

        stepper_arm->enableOutputs();

        stepper_arm->setSpeedInHz(0.6f * ARM_STEPS_PER_RAD);
        stepper_arm->setAcceleration(0.3f * ARM_STEPS_PER_RAD);
    }

    if (servo_gripper) {
        servo_gripper->attach(GRIPPER_PIN);
    }
};

void Manipulator::drive_angle(double fk_theta_base, double fk_theta_arm, double fk_theta_gripper){
    this->theta_base = fk_theta_base;
    this->theta_arm = fk_theta_arm;
    this->theta_gripper = fk_theta_gripper;
    
    if (stepper_base) {
        stepper_base->moveTo(base_rad_to_steps(fk_theta_base));
    };
    if (stepper_arm) {
        stepper_arm->moveTo(arm_rad_to_steps(fk_theta_arm));
    };
    if (servo_gripper) {
        servo_gripper->write(fk_theta_gripper * 180.0 / PI);
    };
};

bool Manipulator::drive_position(double ik_x, double ik_y, double ik_z){
    this->x = ik_x;
    this->y = ik_y;
    this->z = ik_z;
    
    double radial_distance = sqrt((ik_x * ik_x) + (ik_y * ik_y));

    double reachability_error = pow(radial_distance - ARM_LATERAL_OFFSET, 2) + pow(ik_z - ARM_HEIGHT, 2) - pow(ARM_LENGTH, 2);

    if (fabs(reachability_error) > 1.0E-6) {
        return false;
    }
    
    double ik_theta_base = atan2(ik_y, ik_x);

    double ik_theta_arm = atan2(ik_z - ARM_HEIGHT, radial_distance - ARM_LATERAL_OFFSET);

    double ik_theta_gripper = PI;

    drive_angle(ik_theta_base, ik_theta_arm, ik_theta_gripper);

    return true;
};
