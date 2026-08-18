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

fk_position Manipulator::forward_kinematics(double fk_theta_base, double fk_theta_arm){
    fk_position end_factor_position;
    
    end_factor_position.fk_x = (ARM_LATERAL_OFFSET + (ARM_LENGTH * cos(fk_theta_arm))) * cos(fk_theta_base);
    end_factor_position.fk_y = (ARM_LATERAL_OFFSET + (ARM_LENGTH * cos(fk_theta_arm))) * sin(fk_theta_base);
    end_factor_position.fk_z = ARM_HEIGHT + (ARM_LENGTH * sin(fk_theta_arm));

    return end_factor_position;
}

ik_angle Manipulator::inverse_kinematics(double ik_x, double ik_y, double ik_z){
    ik_angle end_factor_angle;

    double radial_distance = sqrt((ik_x * ik_x) + (ik_y * ik_y));
    
    end_factor_angle.ik_theta_base = atan2(ik_y, ik_x);
    end_factor_angle.ik_theta_arm = atan2(ik_z - ARM_HEIGHT, radial_distance - ARM_LATERAL_OFFSET);

    return end_factor_angle;
}

void Manipulator::drive_angle(double drive_theta_base, double drive_theta_arm, double drive_theta_gripper){
    if (stepper_base) {
        stepper_base->moveTo(base_rad_to_steps(drive_theta_base));
    };
    if (stepper_arm) {
        stepper_arm->moveTo(arm_rad_to_steps(drive_theta_arm));
    };
    if (servo_gripper) {
        servo_gripper->write(drive_theta_gripper * 180.0 / PI);
    };

    this->theta_base = drive_theta_base;
    this->theta_arm = drive_theta_arm;
    this->theta_gripper = drive_theta_gripper;

    fk_position output_position = forward_kinematics(drive_theta_base, drive_theta_arm);

    this->x = output_position.fk_x;
    this->y = output_position.fk_y;
    this->z = output_position.fk_z;
};

void Manipulator::drive_position(double drive_x, double drive_y, double drive_z){

    double test_radial_distance = sqrt((drive_x * drive_x) + (drive_y * drive_y));

    double reachability_error = pow(test_radial_distance - ARM_LATERAL_OFFSET, 2) + pow(drive_z - ARM_HEIGHT, 2) - pow(ARM_LENGTH, 2);

    if (fabs(reachability_error) > 1.0E-6) {
        return;
    }

    ik_angle output_angle = inverse_kinematics(drive_x, drive_y, drive_z);

    drive_angle(output_angle.ik_theta_base, output_angle.ik_theta_arm, PI);
};
