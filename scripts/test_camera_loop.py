#!/usr/bin/env python3

import cv2
import time

CAMERA_DEVICE = "/dev/video0"

cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("Camera failed to open")
    exit()

print("Camera opened successfully")
print("Reading frames continuously. Press CTRL+C to stop.")

frame_count = 0
start_time = time.time()

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Frame read failed")
            continue

        frame_count += 1

        elapsed = time.time() - start_time

        if elapsed >= 1.0:
            fps = frame_count / elapsed
            print(f"FPS: {fps:.2f} | Frame shape: {frame.shape}")

            frame_count = 0
            start_time = time.time()

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    cap.release()
    print("Camera released")