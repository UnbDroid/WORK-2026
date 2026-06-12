#include <Arduino.h>
#include "Config.h"

#include <Servo.h>

Servo Garra;

void stepTo(int steps, bool direction, int time, int motor);

void setup() {
  pinMode(Step_M1_Pin, OUTPUT);
  pinMode(Dir_M1_Pin, OUTPUT);
  pinMode(Step_M2_Pin, OUTPUT);
  pinMode(Dir_M2_Pin, OUTPUT);

  Garra.attach(Garra_Pin);
  Garra.write(0);

  delay(1000);
}

void loop() {
  stepTo(200, true, 10000, 2);
  Garra.write(0);
  delay(10000);

  stepTo(200, false, 10000, 2);
  Garra.write(180);
  delay(10000);
}

void stepTo(int steps, bool direction, int time, int motor) {
  switch(motor){
    case 0:
      digitalWrite(Dir_M1_Pin, direction);
      for (int x = 0; x < steps; x++) {
        digitalWrite(Step_M1_Pin, HIGH);
        delayMicroseconds(time);
        digitalWrite(Step_M1_Pin, LOW);
        delayMicroseconds(time);
      }
      break;
    
    case 1:
      digitalWrite(Dir_M2_Pin, direction);
      
      for (int x = 0; x < steps; x++) {
        digitalWrite(Step_M2_Pin, HIGH);
        delayMicroseconds(time);
        digitalWrite(Step_M2_Pin, LOW);
        delayMicroseconds(time);
      }
      break;

    default:
      digitalWrite(Dir_M1_Pin, direction);
      digitalWrite(Dir_M2_Pin, direction);

      for (int x = 0; x < steps; x++) {
        
        digitalWrite(Step_M1_Pin, HIGH);
        digitalWrite(Step_M2_Pin, HIGH);

        delayMicroseconds(time);

        digitalWrite(Step_M1_Pin, LOW);
        digitalWrite(Step_M2_Pin, LOW);

        delayMicroseconds(time);
      }
      break;
  }
}