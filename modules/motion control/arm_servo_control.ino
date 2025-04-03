## ⚙️ Arm Servo Control - `arm_servo_control.ino`
```cpp
#include <Servo.h>

Servo armServo; // Main arm servo
int inputAngle; // Stores input angle from serial

void setup() {
  Serial.begin(9600);           // Start serial communication
  armServo.attach(9);           // Connect servo to pin 9
  Serial.println("Arm Servo Ready");
}

void loop() {
  if (Serial.available() > 0) {
    inputAngle = Serial.parseInt();
    if (inputAngle >= 0 && inputAngle <= 180) {
      armServo.write(inputAngle); // Set servo position
      Serial.print("Servo angle set to: ");
      Serial.println(inputAngle);
    } else {
      Serial.println("Invalid angle. Must be 0-180.");
    }
  }
}
