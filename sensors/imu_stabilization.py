## ⚖️ IMU-Based Stabilization - `imu_stabilization.py`
```python
import smbus2
import time

# MPU6050 I2C address and registers
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

def read_word(bus, addr):
    high = bus.read_byte_data(MPU_ADDR, addr)
    low = bus.read_byte_data(MPU_ADDR, addr + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def initialize_imu():
    bus = smbus2.SMBus(1)
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)
    print("[IMU] MPU6050 Initialized")
    return bus

def read_accelerometer(bus):
    accel_x = read_word(bus, ACCEL_XOUT_H)
    accel_y = read_word(bus, ACCEL_XOUT_H + 2)
    accel_z = read_word(bus, ACCEL_XOUT_H + 4)
    return accel_x, accel_y, accel_z

def main():
    bus = initialize_imu()
    while True:
        x, y, z = read_accelerometer(bus)
        print(f"[IMU] X: {x}, Y: {y}, Z: {z}")
        # Add stabilization logic here (balance correction, alerts, etc.)
        time.sleep(0.5)

if __name__ == '__main__':
    main()
```

This script reads real-time motion data from an **MPU6050 sensor** (gyroscope + accelerometer), allowing your suit to react to balance shifts or sudden movement — perfect for stabilization features like **anti-fall correction** or **auto-leveling limbs**.

---
