#include <Arduino.h>

#define LED_PIN 2 
void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(115200);
  Serial.println("Ola, eu sou o ESP32!");
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  Serial.println("LED Ligado");
  delay(1000); 
  digitalWrite(LED_PIN, LOW);  
  Serial.println("LED Desligado");
  delay(1000);
}