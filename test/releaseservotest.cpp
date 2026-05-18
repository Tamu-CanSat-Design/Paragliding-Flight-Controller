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
  // Sweep 0 → 180
  Serial.println("Moving to 0°");
  myServo.write(0);
  delay(1000);

  Serial.println("Moving to 90°");
  myServo.write(90);
  delay(1000);

  Serial.println("Moving to 180°");
  myServo.write(180);
  delay(1000);

  Serial.println("Moving back to 90°");
  myServo.write(90);
  delay(1000);
}