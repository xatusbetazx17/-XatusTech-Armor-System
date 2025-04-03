## 🧠 Face Detection Vision - `face_detection.py`
```python
import cv2

def initialize_camera():
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        raise IOError("Cannot access camera. Check connection or permissions.")
    return cam

def load_cascade():
    return cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

def detect_faces(frame, cascade):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
    return faces

def main():
    cam = initialize_camera()
    face_cascade = load_cascade()
    print("[Vision] Face detection module initialized.")

    while True:
        ret, frame = cam.read()
        if not ret:
            print("[Vision] Camera frame not received.")
            break

        faces = detect_faces(frame, face_cascade)
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(frame, 'Face', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        cv2.imshow("XatusTech Helmet View - Face Detection", frame)
        key = cv2.waitKey(1)
        if key == ord('q'):
            print("[Vision] Shutting down face detection.")
            break

    cam.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```
