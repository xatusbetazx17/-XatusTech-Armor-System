## ✋ Flex Sensor Input - `flex_sensor.ino`
```cpp
const int flexPin = A0;   // Analog pin A0
int flexValue = 0;        // Stores flex sensor value

void setup() {
  Serial.begin(9600);
  pinMode(flexPin, INPUT);
  Serial.println("Flex Sensor Initialized");
}

void loop() {
  flexValue = analogRead(flexPin);
  Serial.print("Flex Value: ");
  Serial.println(flexValue);
  delay(100);

  // Trigger actions based on flex sensor thresholds
  if (flexValue < 300) {
    Serial.println("[Gesture] Fist Detected");
    // Trigger punch animation or tool switch
  } else if (flexValue > 700) {
    Serial.println("[Gesture] Open Hand Detected");
    // Trigger palm scan or energy shield
  }
}
```
