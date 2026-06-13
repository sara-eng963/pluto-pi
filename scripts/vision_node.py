#!/usr/bin/env python3

import cv2
import time
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Bool

from ai_edge_litert.interpreter import Interpreter


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")

        # =========================
        # Paths
        # =========================
        self.camera_device = "/dev/video0"
        self.model_path = "/home/pluto/design_ws/src/pluto/models/fruit_model.tflite"
        self.classes_path = "/home/pluto/design_ws/src/pluto/models/classes.txt"

        # =========================
        # ROI settings
        # Camera frame is 640x480
        # =========================
        self.roi_x = 160
        self.roi_y = 80
        self.roi_w = 420
        self.roi_h = 380

        # =========================
        # Detection settings
        # =========================
        self.confidence_threshold = 0.70
        self.confirm_required = 5

        self.ordered_fruit = ""

        self.last_label = None
        self.confirm_count = 0
        self.confidence_buffer = []

        self.detected_fruit = "Unknown"
        self.detected_confidence = 0.0
        self.valid = False

        # =========================
        # ROS interfaces
        # =========================
        self.order_sub = self.create_subscription(
            String,
            "/ordered_fruit",
            self.ordered_fruit_callback,
            10
        )

        self.detected_fruit_pub = self.create_publisher(
            String,
            "/detected_fruit",
            10
        )

        self.detected_confidence_pub = self.create_publisher(
            Float32,
            "/detected_confidence",
            10
        )

        self.valid_pub = self.create_publisher(
            Bool,
            "/valid",
            10
        )

        # =========================
        # Load classes
        # =========================
        with open(self.classes_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines() if line.strip()]

        self.get_logger().info("Loaded classes:")
        for i, name in enumerate(self.classes):
            self.get_logger().info(f"{i}: {name}")

        # =========================
        # Load TFLite model
        # =========================
        self.interpreter = Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()

        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()

        self.input_shape = self.input_details[0]["shape"]
        self.input_dtype = self.input_details[0]["dtype"]

        self.model_h = int(self.input_shape[1])
        self.model_w = int(self.input_shape[2])

        self.get_logger().info(f"Model input shape: {self.input_shape}")
        self.get_logger().info(f"Model input dtype: {self.input_dtype}")
        self.get_logger().info(f"Model input size: {self.model_w} x {self.model_h}")

        # =========================
        # Open camera
        # =========================
        self.cap = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error("Camera failed to open")
            raise RuntimeError("Camera failed to open")

        self.get_logger().info("Camera opened successfully")
        self.get_logger().info("Vision node started")
        self.get_logger().info("Waiting for /ordered_fruit...")

        # =========================
        # FPS/debug
        # =========================
        self.frame_count = 0
        self.start_time = time.time()

        # 20 Hz processing timer
        self.timer = self.create_timer(0.05, self.process_frame)

    def ordered_fruit_callback(self, msg):
        new_order = msg.data.strip()

        self.ordered_fruit = new_order

        # Reset confirmation state when order changes
        self.last_label = None
        self.confirm_count = 0
        self.confidence_buffer = []

        self.detected_fruit = "Unknown"
        self.detected_confidence = 0.0
        self.valid = False

        self.get_logger().info(f"Received ordered fruit: {self.ordered_fruit}")

    def preprocess_roi(self, frame):
        roi = frame[
            self.roi_y:self.roi_y + self.roi_h,
            self.roi_x:self.roi_x + self.roi_w
        ]

        if roi.size == 0:
            return None

        roi_rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
        resized = cv2.resize(roi_rgb, (self.model_w, self.model_h))

        input_data = np.expand_dims(resized, axis=0)

        if self.input_dtype == np.float32:
            input_data = input_data.astype(np.float32) / 255.0
        else:
            input_data = input_data.astype(self.input_dtype)

        return input_data

    def run_inference(self, input_data):
        self.interpreter.set_tensor(self.input_details[0]["index"], input_data)
        self.interpreter.invoke()

        output = self.interpreter.get_tensor(self.output_details[0]["index"])[0]

        class_id = int(np.argmax(output))
        confidence = float(output[class_id])

        if class_id < len(self.classes):
            label = self.classes[class_id]
        else:
            label = f"class_{class_id}"

        return label, confidence, output

    def update_confirmation_filter(self, label, confidence):
        # Low confidence = Unknown
        if confidence < self.confidence_threshold:
            label = "Unknown"

        if label == self.last_label:
            self.confirm_count += 1
            self.confidence_buffer.append(confidence)
        else:
            self.last_label = label
            self.confirm_count = 1
            self.confidence_buffer = [confidence]

        if self.confirm_count >= self.confirm_required:
            self.detected_fruit = label
            self.detected_confidence = float(np.mean(self.confidence_buffer[-self.confirm_required:]))

            if (
                self.ordered_fruit != ""
                and self.detected_fruit == self.ordered_fruit
                and self.detected_fruit != "Unknown"
            ):
                self.valid = True
            else:
                self.valid = False

            self.publish_detection()

    def publish_detection(self):
        fruit_msg = String()
        fruit_msg.data = self.detected_fruit

        confidence_msg = Float32()
        confidence_msg.data = float(self.detected_confidence)

        valid_msg = Bool()
        valid_msg.data = bool(self.valid)

        self.detected_fruit_pub.publish(fruit_msg)
        self.detected_confidence_pub.publish(confidence_msg)
        self.valid_pub.publish(valid_msg)

    def process_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Frame read failed")
            return

        input_data = self.preprocess_roi(frame)

        if input_data is None:
            self.get_logger().warn("ROI crop failed")
            return

        label, confidence, output = self.run_inference(input_data)

        self.update_confirmation_filter(label, confidence)

        self.frame_count += 1
        elapsed = time.time() - self.start_time

        if elapsed >= 1.0:
            fps = self.frame_count / elapsed

            self.get_logger().info(
                f"FPS: {fps:.2f} | "
                f"Order: {self.ordered_fruit if self.ordered_fruit else 'None'} | "
                f"Current: {label} ({confidence:.3f}) | "
                f"Confirmed: {self.detected_fruit} ({self.detected_confidence:.3f}) | "
                f"Valid: {self.valid} | "
                f"Count: {self.confirm_count}/{self.confirm_required}"
            )

            self.frame_count = 0
            self.start_time = time.time()

    def destroy_node(self):
        if hasattr(self, "cap") and self.cap is not None:
            self.cap.release()
            self.get_logger().info("Camera released")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = VisionNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"Vision node error: {e}")

    finally:
        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()