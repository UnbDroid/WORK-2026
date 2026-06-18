#include <Arduino.h>
#include "Config.h"

#include <ESP32Servo.h>
#include <FastAccelStepper.h>

FastAccelStepperEngine engine;
FastAccelStepper *stepper_arm = nullptr;
FastAccelStepper *stepper_base = nullptr;
Servo Gripper;

void setup() {

  Gripper.attach(GRIPPER_PIN);

  pinMode(MS1_M1_PIN, OUTPUT);
  pinMode(MS2_M1_PIN, OUTPUT);
  pinMode(MS3_M1_PIN, OUTPUT);
  pinMode(MS1_M2_PIN, OUTPUT);
  pinMode(MS2_M2_PIN, OUTPUT);
  pinMode(MS3_M2_PIN, OUTPUT);

  digitalWrite(MS1_M1_PIN, HIGH);
  digitalWrite(MS2_M1_PIN, HIGH);
  digitalWrite(MS3_M1_PIN, HIGH);
  digitalWrite(MS1_M2_PIN, HIGH);
  digitalWrite(MS2_M2_PIN, HIGH);
  digitalWrite(MS3_M2_PIN, HIGH);

  engine.init();

  stepper_base = engine.stepperConnectToPin(STEP_M1_PIN);
  stepper_arm = engine.stepperConnectToPin(STEP_M2_PIN);

  if (stepper_base) {
    stepper_base->setDirectionPin(DIR_M1_PIN);
    stepper_base->setEnablePin(ENABLE_M1_PIN);
    stepper_base->setAutoEnable(true);

    stepper_base->setSpeedInHz((1.0f * STEPS_PER_REV_BASE * MICROSTEPPING_MULT) / (2.0f * PI));
    stepper_base->setAcceleration((1.5f * STEPS_PER_REV_BASE * MICROSTEPPING_MULT) / (2.0f * PI));
  }

  if (stepper_arm) {
    stepper_arm->setDirectionPin(DIR_M2_PIN);
    stepper_base->setEnablePin(ENABLE_M1_PIN);
    stepper_base->setAutoEnable(false);

    stepper_arm->setSpeedInHz((1.0f * STEPS_PER_REV_ARM * MICROSTEPPING_MULT) / (2.0f * PI));
    stepper_arm->setAcceleration((1.5f * STEPS_PER_REV_ARM * MICROSTEPPING_MULT) / (2.0f * PI));
  }

  stepper_base->moveTo(22400);
  stepper_arm->moveTo(1395);
}

void loop() {

}