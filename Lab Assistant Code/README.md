
# 🤖 Single Person Lock & Follow Robot (YOLOv8)

This project implements a **single-person locking and following robot system** using **YOLOv8**, **OpenCV**, and **Serial communication** (ESP32 based motor controller).  
The robot detects **only one person**, locks onto them, and follows while maintaining distance and alignment.

---

## ✨ Features

- 🎯 **Single Person Lock**
  - Automatically locks onto the **largest detected person**
  - Ignores other people
  - Unlocks if the target is lost for a defined timeout

- 🧠 **YOLOv8 Tracking**
  - Uses `YOLOv8n` for fast, real-time detection
  - Built-in object ID tracking

- 🎥 **Low-Latency Camera Pipeline**
  - Reduced frame buffering
  - Optimized for embedded systems (Jetson / Raspberry Pi)

- 📈 **Kalman Filter**
  - Smooths horizontal movement
  - Reduces jitter in steering commands

- 🚗 **Distance Control**
  - Forward / Stop / Reverse based on bounding box size

- 🔁 **Command Stability Filter**
  - Prevents command flickering
  - Sends commands only after consistency


---

## 🧑‍💻 Software Requirements

- Python **3.8+**
- OpenCV
- PySerial
- NumPy
- Ultralytics YOLO

Install dependencies:

```bash
pip install ultralytics opencv-python pyserial numpy
```

---

## 📁 Project Structure

```
.
├── car.py
├── yolov8n.pt
├── yolov8n.onnx
└── venv
```

---

## ⚙️ Configuration

### Serial Settings
```python
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200
```

### Camera Settings
```python
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
```

### Control Parameters
```python
CENTER_TOLERANCE = 80
STABILITY_COUNT = 3
LOCK_TIMEOUT = 2.0
```

### Distance Thresholds
```python
FORWARD_THRESHOLD = 270
STOP_THRESHOLD = 350
REVERSE_THRESHOLD = 420
```

---

## 🎮 Robot Command Mapping

| Command | Action |
|-------|--------|
| W | Move Forward |
| S | Stop |
| R | Hold distance |
| F | Fast Right |
| f | Slow Right |
| B | Fast Left |
| b | Slow Left |


---

## ▶️ Running the Project

```bash
# For creating virtual Enviornment
python -m venv venv

# For Activating venv
source venv/bin/activate

# For running main code
python car.py

```


---

## 🔒 Locking Logic

1. Detect all persons
2. Select **largest bounding box**
3. Lock onto its YOLO tracking ID
4. Follow only that ID
5. Unlock if lost for `LOCK_TIMEOUT` seconds

---

## 🧪 Performance Tips

- Use **YOLOv8n** for best speed
- Lower `FRAME_WIDTH` for slower hardware
- Increase `STABILITY_COUNT` if motors jitter
- Tune distance thresholds for your camera FOV

---

## 🛑 Safe Shutdown

On exit:
- Robot sends **STOP command**
- Serial port closed
- Camera released

---
