## 🧠 Motion Command Client - `move_arm.py`
```python
import serial
import time

class ArmController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        self.arduino = serial.Serial(port, baudrate)
        time.sleep(2)  # Allow time to establish connection
        print("[Python] Arm controller connected.")

    def move_arm(self, angle):
        if 0 <= angle <= 180:
            self.arduino.write(f"{angle}\n".encode())
            print(f"[Python] Moving arm to {angle} degrees.")
        else:
            print("[Python] Invalid angle. Must be between 0 and 180.")

if __name__ == '__main__':
    controller = ArmController()
    controller.move_arm(90)  # Test move to 90 degrees
```
