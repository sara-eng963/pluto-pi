#!/usr/bin/env python3

import cv2
import time
import numpy as np
from ai_edge_litert.interpreter import Interpreter

CAMERA_DEVICE = "/dev/video0"

MODEL_PATH = "/home/pluto/design_ws/src/pluto/models/fruit_model.tflite"
CLASSES_PATH = "/home/pluto/design_ws/src/pluto/models/classes.txt"

# ROI settings
# Camera frame is 640x480
# x = horizontal direction
# y = vertical direction
ROI_X = 160
ROI_Y = 80
ROI_W = 420
ROI_H = 380

# Load class names
with open(CLASSES_PATH, "r") as f:
    classes = [line.strip() for line in f.readlines() if line.strip()]

print("Classes:")
for i, name in enumerate(classes):
    print(f"{i}: {name}")

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
saved_debug = False

try:
    while True:
        ret, frame = cap.read()

        if not ret:
            print("Frame read failed")
            continue

        # Crop ROI from full frame
        roi = frame[ROI_Y:ROI_Y + ROI_H, ROI_X:ROI_X + ROI_W]

        # Safety check
        if roi.size == 0:
            print("ROI crop failed. Check ROI values.")
            continue

        # Convert BGR to RGB because OpenCV uses BGR and most models expect RGB
        roi_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)

        # Resize ROI to model input size
        resized = cv2.resize(roi_rgb, (model_w, model_h))

        # Save debug images once
        if not saved_debug:
            frame_with_roi = frame.copy()

            cv2.rectangle(
                frame_with_roi,
                (ROI_X, ROI_Y),
                (ROI_X + ROI_W, ROI_Y + ROI_H),
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame_with_roi,
                "ROI",
                (ROI_X, ROI_Y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

            debug_model_input_bgr = cv2.cvtColor(resized, cv2.COLOR_RGB2BGR)

            cv2.imwrite("debug_full_frame_with_roi.jpg", frame_with_roi)
            cv2.imwrite("debug_roi_raw.jpg", roi)
            cv2.imwrite("debug_model_input.jpg", debug_model_input_bgr)

            print("Saved debug_full_frame_with_roi.jpg")
            print("Saved debug_roi_raw.jpg")
            print("Saved debug_model_input.jpg")

            saved_debug = True

        # Add batch dimension: (224, 224, 3) -> (1, 224, 224, 3)
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

        # Get prediction
        class_id = int(np.argmax(output))
        confidence = float(output[class_id])

        if class_id < len(classes):
            label = classes[class_id]
        else:
            label = f"class_{class_id}"

        # FPS counting
        frame_count += 1
        elapsed = time.time() - start_time

        # Print once per second
        if elapsed >= 1.0:
            fps = frame_count / elapsed

            print("\nRaw output:", output)

            for i, score in enumerate(output):
                name = classes[i] if i < len(classes) else f"class_{i}"
                print(f"{i}: {name} = {float(score):.4f}")

            print(f"FPS: {fps:.2f} | ROI Prediction: {label} | Confidence: {confidence:.3f}")

            frame_count = 0
            start_time = time.time()

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    cap.release()
    print("Camera released")