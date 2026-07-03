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

  robot_arm.drive_position(0.189333, 0.327933, 0.2075);

  printf("X: %.6lf\n", robot_arm.get_x_position());
  printf("Y: %.6lf\n", robot_arm.get_y_position());
  printf("Z: %.6lf\n", robot_arm.get_z_position());
  printf("Theta_Base: %.6lf\n", robot_arm.get_base_angle());
  printf("Theta_Arm: %.6lf\n", robot_arm.get_arm_angle());
  printf("Theta_Gripper: %.6lf", robot_arm.get_gripper_angle());
}

void loop() {}