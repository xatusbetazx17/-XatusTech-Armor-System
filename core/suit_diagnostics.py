## 🔍 Suit Diagnostics Module - `suit_diagnostics.py`
```python
import psutil
import platform
import datetime

class SuitDiagnostics:
    def __init__(self):
        self.system = platform.system()
        self.release = platform.release()
        self.processor = platform.processor()
        self.boot_time = datetime.datetime.fromtimestamp(psutil.boot_time())

    def get_cpu_usage(self):
        return psutil.cpu_percent(interval=1)

    def get_memory_status(self):
        memory = psutil.virtual_memory()
        return {
            "total": memory.total,
            "available": memory.available,
            "used": memory.used,
            "percent": memory.percent
        }

    def display_diagnostics(self):
        print("=== XatusTech Suit Diagnostics ===")
        print(f"System: {self.system} {self.release}")
        print(f"Processor: {self.processor}")
        print(f"Boot Time: {self.boot_time.strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"CPU Usage: {self.get_cpu_usage()}%")
        memory = self.get_memory_status()
        print("Memory Usage:")
        print(f" - Total: {memory['total'] / (1024**3):.2f} GB")
        print(f" - Used: {memory['used'] / (1024**3):.2f} GB")
        print(f" - Available: {memory['available'] / (1024**3):.2f} GB")
        print(f" - Usage: {memory['percent']}%")
        print("==================================")

if __name__ == '__main__':
    diagnostics = SuitDiagnostics()
    diagnostics.display_diagnostics()
