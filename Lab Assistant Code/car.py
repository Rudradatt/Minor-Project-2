#!/usr/bin/env python3
import cv2
import serial
import time
import numpy as np
from ultralytics import YOLO

# ================== SETTINGS ==================
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

FRAME_WIDTH = 320
FRAME_HEIGHT = 240
CENTER_TOLERANCE = 80
STABILITY_COUNT = 3
IDLE_TIMEOUT = 1
CONTROL_DELAY = 0.13

# Distance control
FORWARD_THRESHOLD = 270
STOP_THRESHOLD = 350
REVERSE_THRESHOLD = 420

# Locking
LOCK_TIMEOUT = 2.0  # seconds before unlocking target

# ================== SERIAL ==================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5)
    print("✅ Serial connected:", SERIAL_PORT)
except Exception as e:
    ser = None
    print("❌ Serial failed:", e)

# ================== YOLO ==================
model = YOLO("yolov8n.pt")

# ================== CAMERA ==================
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
cap.set(cv2.CAP_PROP_FPS, 30)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
CENTER_X = actual_width // 2

print(f"🎥 Camera {actual_width}x{actual_height}")

# ================== KALMAN FILTER ==================
kf = cv2.KalmanFilter(2, 1)
kf.measurementMatrix = np.array([[1, 0]], np.float32)
kf.transitionMatrix = np.array([[1, 1], [0, 1]], np.float32)
kf.processNoiseCov = np.eye(2, dtype=np.float32) * 0.03
kf.measurementNoiseCov = np.array([[1]], np.float32) * 0.5

cx_filtered = None

# ================== STATE ==================
last_cmd = 'S'
stable_cmd = 'S'
same_cmd_count = 0
last_seen_time = time.time()

locked_id = None
last_lock_time = time.time()

# ================== SERIAL SEND ==================
def send_command(cmd):
    global last_cmd
    if ser and cmd != last_cmd:
        ser.write(cmd.encode())
        last_cmd = cmd
        print("➡ CMD:", cmd)

# ================== MAIN LOOP ==================
try:
    while True:
        cap.grab()
        cap.grab()

        ret, frame = cap.read()
        if not ret:
            time.sleep(0.05)
            continue

        cmd = 'S'

        results = model.track(
            frame,
            classes=[0],
            conf=0.55,
            imgsz=320,
            persist=True,
            verbose=False
        )

        if results and results[0].boxes and results[0].boxes.id is not None:
            boxes = results[0].boxes
            ids = boxes.id.cpu().numpy().astype(int)
            xyxy = boxes.xyxy.cpu().numpy()

            # === LOCK TARGET ===
            if locked_id is None:
                areas = [(x2-x1)*(y2-y1) for (x1,y1,x2,y2) in xyxy]
                idx = np.argmax(areas)
                locked_id = ids[idx]
                last_lock_time = time.time()
                print(f"🔒 Locked on ID {locked_id}")

            # === FIND LOCKED PERSON ONLY ===
            found = False
            for i, pid in enumerate(ids):
                if pid == locked_id:
                    x1, y1, x2, y2 = xyxy[i].astype(int)
                    found = True
                    last_seen_time = time.time()
                    last_lock_time = time.time()
                    break

            if not found:
                if time.time() - last_lock_time > LOCK_TIMEOUT:
                    print("🔓 Target lost")
                    locked_id = None
                send_command('S')
                continue

            # === POSITION ===
            cx = (x1 + x2) // 2
            box_height = y2 - y1

            measured = np.array([[np.float32(cx)]])
            if cx_filtered is None:
                kf.statePre = np.array([[cx], [0]], np.float32)

            kf.correct(measured)
            cx_filtered = kf.predict()[0][0]

            deviation = cx_filtered - CENTER_X
            abs_dev = abs(deviation)

            # === CONTROL LOGIC ===
            if abs_dev <= CENTER_TOLERANCE:
                if box_height < FORWARD_THRESHOLD:
                    cmd = 'W'
                elif box_height > REVERSE_THRESHOLD:
                    cmd = 'S'
                elif box_height > STOP_THRESHOLD:
                    cmd = 'S'
                else:
                    cmd = 'R'
            else:
                if abs_dev <= 100:
                    cmd = 'b' if deviation < 0 else 'f'
                else:
                    cmd = 'B' if deviation < 0 else 'F'

            # === VISUALS ===
            cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
            cv2.putText(frame, f"LOCKED ID {locked_id}", (10,20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        elif time.time() - last_seen_time > IDLE_TIMEOUT:
            cmd = 'S'
            locked_id = None

        # === STABILITY FILTER ===
        if cmd == stable_cmd:
            same_cmd_count += 1
        else:
            same_cmd_count = 0
            stable_cmd = cmd

        if same_cmd_count >= STABILITY_COUNT:
            send_command(cmd)

        cv2.imshow("Single Person Lock - YOLO Follower", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(CONTROL_DELAY)

except KeyboardInterrupt:
    print("\n🛑 Exit")

finally:
    send_command('S')
    if ser:
        ser.close()
    cap.release()
    cv2.destroyAllWindows()
    print("🔚 Clean shutdown")
