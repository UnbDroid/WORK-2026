#include <Arduino.h>
#include <Servo.h>

// Conexões do driver A4988
#define dirPin1 14
#define stepPin1 13
#define dirPin2 32
#define stepPin2 33

Servo servo;

void stepTo(int steps, bool direction, int time, int motor);

void setup() {
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);

  servo.attach(26);
  servo.write(0);
  delay(1000);
}

void loop() {
  stepTo(200, true, 2000, 2);
  servo.write(0);
  delay(2000);

  stepTo(200, false, 2000, 2);
  servo.write(180);
  delay(2000);
}

// São 200 passos (steps) por revolução
// direction = true: sentido horário; direction = false: sentido anti-horário
// time indica o tempo de espera entre os passos (de certo modo a velocidade) 2000
// motor = 0: liga um motor; motor = 1: liga o outro motor; motor = 2: liga os dois motores
void stepTo(int steps, bool direction, int time, int motor) {
  switch(motor){
    case 0:
      digitalWrite(dirPin1, direction);
      for (int x = 0; x < steps; x++) {
        digitalWrite(stepPin1, HIGH);
        delayMicroseconds(time);
        digitalWrite(stepPin1, LOW);
        delayMicroseconds(time);
      }
      break;
    case 1:
      digitalWrite(dirPin2, direction);
      for (int x = 0; x < steps; x++) {
        digitalWrite(stepPin2, HIGH);
        delayMicroseconds(time);
        digitalWrite(stepPin2, LOW);
        delayMicroseconds(time);
      }
      break;
    default:
      digitalWrite(dirPin1, direction);
      digitalWrite(dirPin2, direction);
      for (int x = 0; x < steps; x++) {
        digitalWrite(stepPin1, HIGH);
        digitalWrite(stepPin2, HIGH);
        delayMicroseconds(time);
        digitalWrite(stepPin1, LOW);
        digitalWrite(stepPin2, LOW);
        delayMicroseconds(time);
      }
      break;
  }
}