#include <Servo.h>
#include <Arduino.h>
Servo myServo;

const int SERVO_PIN = 15;

void setup() {
  myServo.attach(SERVO_PIN, 500, 2400);  // min/max pulse widths for MG90S
  myServo.write(90);                     // move to center on startup
}

void loop() {
  // Command a specific angle (0–180)
  myServo.write(0);
  delay(1000);

  myServo.write(90);
  delay(1000);

  myServo.write(180);
  delay(1000);
}