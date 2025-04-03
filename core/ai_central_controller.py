import time
import random
from modules.motion_control.move_arm import move_arm
from modules.voice_control.voice_command import activate_buster

class XatusAIController:
    def __init__(self):
        self.status = {
            "power_level": 100,
            "temp_celsius": 36.5,
            "mode": "IDLE",
            "arm_ready": True,
            "voice_system": True,
            "vision_active": False
        }

    def run_diagnostics(self):
        print("[AI] Running initial diagnostics...")
        for key, value in self.status.items():
            print(f"[AI] {key}: {value}")
        print("[AI] Diagnostics complete. All systems green.")

    def update_status(self):
        self.status["power_level"] -= random.uniform(0.1, 0.5)
        self.status["temp_celsius"] += random.uniform(-0.2, 0.3)

    def enter_combat_mode(self):
        print("[AI] Switching to COMBAT mode.")
        self.status["mode"] = "COMBAT"
        move_arm(45)  # Prepping buster position
        activate_buster()

    def monitor_environment(self):
        print("[AI] Monitoring environment... no threats detected.")

    def shutdown_sequence(self):
        print("[AI] Powering down systems...")
        self.status["mode"] = "SHUTDOWN"
        print("[AI] All modules offline.")

if __name__ == '__main__':
    controller = XatusAIController()
    controller.run_diagnostics()

    for cycle in range(5):
        time.sleep(1)
        controller.update_status()
        print(f"[AI] Status Update: {controller.status}")

    controller.enter_combat_mode()
    controller.monitor_environment()
    controller.shutdown_sequence()
