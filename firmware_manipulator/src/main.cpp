#include <Arduino.h>
#include "Config.h"
#include "Arm.h"

#include <ESP32Servo.h>
#include <FastAccelStepper.h>

FastAccelStepperEngine engine;
FastAccelStepper *stepper_base = nullptr;
FastAccelStepper *stepper_arm = nullptr;
Servo gripper;

Manipulator robot_arm;

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

  engine.init();

  stepper_base = engine.stepperConnectToPin(STEP_M1_PIN);
  stepper_arm = engine.stepperConnectToPin(STEP_M2_PIN);
  
  robot_arm.init(stepper_base, stepper_arm, &gripper);

  delay(2000);

  robot_arm.drive_position(0.236021, 0.236021, 0.375000);
}

void loop() {

}