#include <Servo.h>
#include <Arduino.h>
Servo myServo;

const int SERVO_PIN = 15;

void setup() {
  Serial.begin(115200);
  myServo.attach(SERVO_PIN);
  Serial.println("Servo test starting...");
}

void loop() {
  
  Serial.println("Moving to 0°");
  myServo.write(20);
  delay(1000);

}