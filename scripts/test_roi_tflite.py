#!/usr/bin/env python3

import cv2
import time
import numpy as np
from ai_edge_litert.interpreter import Interpreter

CAMERA_DEVICE = "/dev/video0"

MODEL_PATH = "/home/pluto/design_ws/src/pluto/models/fruit_model.tflite"
CLASSES_PATH = "/home/pluto/design_ws/src/pluto/models/classes.txt"

# ROI settings
ROI_X = 160
ROI_Y = 80
ROI_W = 420
ROI_H = 380

# Load class names
with open(CLASSES_PATH, "r") as f:
    classes = [line.strip() for line in f.readlines() if line.strip()]

# Load TFLite model
interpreter = Interpreter(model_path=MODEL_PATH)
interpreter.allocate_tensors()

input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

input_shape = input_details[0]["shape"]
input_dtype = input_details[0]["dtype"]

model_h = int(input_shape[1])
model_w = int(input_shape[2])

print("Model input shape:", input_shape)
print("Model input dtype:", input_dtype)
print("Model input size:", model_w, "x", model_h)

# Open camera
cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("Camera failed to open")
    exit()

print("Camera opened successfully")
print("Running ROI TFLite inference. Press CTRL+C to stop.")

frame_count = 0
start_time = time.time()

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Frame read failed")
            continue

        # Crop ROI from full frame
        roi = frame[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]

        # Convert BGR to RGB because OpenCV uses BGR
        roi_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)

        # Resize ROI to model input size
        resized = cv2.resize(roi_rgb, (model_w, model_h))

        # Add batch dimension
        input_data = np.expand_dims(resized, axis=0)

        # Match model input type
        if input_dtype == np.float32:
            input_data = input_data.astype(np.float32) / 255.0
        else:
            input_data = input_data.astype(input_dtype)

        # Run inference
        interpreter.set_tensor(input_details[0]["index"], input_data)
        interpreter.invoke()

        output = interpreter.get_tensor(output_details[0]["index"])[0]

        print("\nRaw output:", output)

        if elapsed >= 1.0:
            fps = frame_count / elapsed

            print("\nRaw output:", output)

            for i, score in enumerate(output):
                name = classes[i] if i < len(classes) else f"class_{i}"
                print(f"{i}: {name} = {float(score):.4f}")

            print(f"FPS: {fps:.2f} | ROI Prediction: {label} | Confidence: {confidence:.3f}")

            frame_count = 0
            start_time = time.time()
        class_id = int(np.argmax(output))
        confidence = float(output[class_id])
        label = classes[class_id]

        frame_count += 1
        elapsed = time.time() - start_time

        if elapsed >= 1.0:
            fps = frame_count / elapsed
            print(f"FPS: {fps:.2f} | ROI Prediction: {label} | Confidence: {confidence:.3f}")

            frame_count = 0
            start_time = time.time()

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    cap.release()
    print("Camera released")