## 💥 Buster Mode Activation - `buster_activation.py`
```python
import time
import random

def charge_buster():
    print("[Buster] Charging initiated...")
    for i in range(1, 6):
        print(f"[Buster] Charge level: {i * 20}%")
        time.sleep(0.2)
    print("[Buster] Charge complete! Ready to fire.")

def fire_buster():
    print("[Buster] Firing energy projectile 💥")
    recoil = random.uniform(0.1, 0.3)
    print(f"[Buster] Recoil detected: {recoil:.2f} units")

def activate_buster():
    print("[Buster] Activation command received.")
    charge_buster()
    fire_buster()

if __name__ == '__main__':
    activate_buster()
```

This script simulates the visual and functional response of activating the arm cannon. You can connect it to GPIO outputs, sound effects, or light modules on the suit.

---
