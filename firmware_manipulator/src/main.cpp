#include <Arduino.h>
#include "Config.h"

#include <ESP32Servo.h>
#include <FastAccelStepper.h>

FastAccelStepperEngine engine;
FastAccelStepper *stepper = nullptr;
Servo Garra;

void setup() {

  Garra.attach(Garra_Pin);

  pinMode(MS1_M1_Pin, OUTPUT);
  pinMode(MS2_M1_Pin, OUTPUT);
  pinMode(MS3_M1_Pin, OUTPUT);

  digitalWrite(MS1_M1_Pin, HIGH);
  digitalWrite(MS2_M1_Pin, HIGH);
  digitalWrite(MS3_M1_Pin, HIGH);

  engine.init();

  stepper = engine.stepperConnectToPin(Step_M1_Pin);

  if (stepper) {
    stepper->setDirectionPin(Dir_M1_Pin);

    stepper->setSpeedInHz((1.0f * STEPS_PER_REV_BASE * MICROSTEPPING_MULT) / (2.0f * PI));
    stepper->setAcceleration((1.5f * STEPS_PER_REV_BASE * MICROSTEPPING_MULT) / (2.0f * PI));

    stepper->moveTo(22400);
  }
}

void loop() {

}