#!/usr/bin/env python3

import cv2

CAMERA_DEVICE = "/dev/video0"

# ROI settings
# Image is 640x480
# x = horizontal direction
# y = vertical direction
ROI_X = 160
ROI_Y = 80
ROI_W = 320
ROI_H = 320

cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("Camera failed to open")
    exit()

# Read one frame
ret, frame = cap.read()

if not ret:
    print("Frame read failed")
    cap.release()
    exit()

print("Frame shape:", frame.shape)

# Crop ROI
roi = frame[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]

# Draw rectangle on full frame
frame_with_roi = frame.copy()
cv2.rectangle(
    frame_with_roi,
    (ROI_X, ROI_Y),
    (ROI_X + ROI_W, ROI_Y + ROI_H),
    (0, 255, 0),
    2
)

# Add label text
cv2.putText(
    frame_with_roi,
    "ROI",
    (ROI_X, ROI_Y - 10),
    cv2.FONT_HERSHEY_SIMPLEX,
    0.8,
    (0, 255, 0),
    2
)

# Save both images
cv2.imwrite("full_frame_with_roi.jpg", frame_with_roi)
cv2.imwrite("roi_crop.jpg", roi)

print("Saved: full_frame_with_roi.jpg")
print("Saved: roi_crop.jpg")

cap.release()
print("Camera released")