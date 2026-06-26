#include <Arduino.h>
#include "Config.h"
#include "Arm.h"

#include <ESP32Servo.h>
#include <FastAccelStepper.h>

FastAccelStepperEngine engine;
FastAccelStepper *stepper_arm = nullptr;
FastAccelStepper *stepper_base = nullptr;
Servo gripper;

Manipulator Arm;

void setup() { 
  
  pinMode(MS1_M1_PIN, OUTPUT);
  pinMode(MS2_M1_PIN, OUTPUT);
  pinMode(MS3_M1_PIN, OUTPUT);

  pinMode(MS1_M2_PIN, OUTPUT);
  pinMode(MS2_M2_PIN, OUTPUT);
  pinMode(MS3_M2_PIN, OUTPUT);

  pinMode(ENABLE_M1_PIN, OUTPUT);
  pinMode(ENABLE_M2_PIN, OUTPUT);

  digitalWrite(MS1_M1_PIN, HIGH);
  digitalWrite(MS2_M1_PIN, HIGH);
  digitalWrite(MS3_M1_PIN, HIGH);

  digitalWrite(MS1_M2_PIN, HIGH);
  digitalWrite(MS2_M2_PIN, HIGH);
  digitalWrite(MS3_M2_PIN, HIGH);

  gripper.attach(GRIPPER_PIN);

  engine.init();

  stepper_base = engine.stepperConnectToPin(STEP_M1_PIN);
  stepper_arm = engine.stepperConnectToPin(STEP_M2_PIN);

  if (stepper_base) {
    stepper_base->setDirectionPin(DIR_M1_PIN);
    stepper_base->setEnablePin(ENABLE_M1_PIN);
    stepper_base->setAutoEnable(true);

    stepper_base->setSpeedInHz((1.0f * STEPS_PER_REV_BASE * MICROSTEPPING_MULT) / (2.0f * PI));
    stepper_base->setAcceleration((0.4f * STEPS_PER_REV_BASE * MICROSTEPPING_MULT) / (2.0f * PI));
  }

  if (stepper_arm) {
    stepper_arm->setDirectionPin(DIR_M2_PIN);
    stepper_arm->setEnablePin(ENABLE_M2_PIN);
    stepper_arm->setAutoEnable(false);

    stepper_arm->enableOutputs();

    stepper_arm->setSpeedInHz((0.6f * STEPS_PER_REV_ARM * MICROSTEPPING_MULT) / (2.0f * PI));
    stepper_arm->setAcceleration((0.3f * STEPS_PER_REV_ARM * MICROSTEPPING_MULT) / (2.0f * PI));
  }
  
  Arm.init(stepper_base, stepper_arm, &gripper);

  Arm.drive_angle(PI, PI/6, PI);
}

void loop() {

}