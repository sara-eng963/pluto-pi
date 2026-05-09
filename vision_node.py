import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import tensorflow as tf
import cv2
import numpy as np

class FruitVisionNode(Node):
    def __init__(self):
        super().__init__('fruit_vision_node')

        # ── state ────────────────────────────────────────────────
        self.requested_item = None      # fruit requested by user
        self.vision_active = False      # camera on/off
        self.bridge = CvBridge()
        self.frame_count = 0

        # ── parameters ───────────────────────────────────────────
        self.declare_parameter('model_path', '/home/pluto/design_ws/src/pluto/models/fruit_model.keras')
        self.declare_parameter('classes_path', '/home/pluto/design_ws/src/pluto/models/classes.txt')
        self.declare_parameter('conf_threshold', 0.5)  # LOWERED to 0.5
        self.declare_parameter('roi_size', 320)
        self.declare_parameter('roi_y_ratio', 0.85)
        
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        classes_path = self.get_parameter('classes_path').get_parameter_value().string_value
        self.conf = self.get_parameter('conf_threshold').get_parameter_value().double_value
        self.roi_size = self.get_parameter('roi_size').get_parameter_value().integer_value
        self.roi_y_ratio = self.get_parameter('roi_y_ratio').get_parameter_value().double_value
        
        # Load model
        self.model = tf.keras.models.load_model(model_path)
        self.get_logger().info(f'✅ TensorFlow model loaded: {model_path}')
        
        # Load classes
        with open(classes_path, 'r') as f:
            self.class_names = [line.strip() for line in f if line.strip()]
        self.get_logger().info(f'📋 Classes: {self.class_names}')
        
        # Model input size
        self.img_size = (224, 224)
        
        # ── subscribers ──────────────────────────────────────────
        self.create_subscription(String, '/from_user', self.user_callback, 10)
        self.create_subscription(Bool, '/activate_vision', self.activation_callback, 10)
        self.create_subscription(Image, '/image_raw', self.image_callback, 10)

        # ── publishers ───────────────────────────────────────────
        self.item_pub = self.create_publisher(String, '/item', 10)
        self.det_pub = self.create_publisher(Detection2DArray, '/fruit/detections', 10)
        self.pick_pub = self.create_publisher(PointStamped, '/fruit/pick_target', 10)
        self.viz_pub = self.create_publisher(Image, '/fruit/visualization', 10)

        self.get_logger().info('🍎 Fruit Vision Node ready. Waiting for activation...')

    def user_callback(self, msg):
        self.requested_item = msg.data.strip().lower()
        self.get_logger().info(f'🎯 Requested item set to: {self.requested_item}')

    def activation_callback(self, msg):
        self.vision_active = msg.data
        state = 'ON' if msg.data else 'OFF'
        self.get_logger().info(f'📷 Vision activated: {state}')

    def get_roi(self, frame):
        h, w = frame.shape[:2]
        
        roi_half = self.roi_size // 2
        center_x = w // 2
        
        x1 = center_x - roi_half
        x2 = center_x + roi_half
        y2 = int(h * self.roi_y_ratio)
        y1 = y2 - self.roi_size
        
        x1 = max(0, x1)
        x2 = min(w, x2)
        y1 = max(0, y1)
        y2 = min(h, y2)
        
        roi = frame[y1:y2, x1:x2]
        return roi, (x1, y1, x2, y2)

    def classify_roi(self, roi):
        if roi.size == 0:
            return "unknown", 0.0
        
        rgb = cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, self.img_size)
        x = np.expand_dims(rgb, axis=0).astype(np.float32) / 255.0
        
        probs = self.model.predict(x, verbose=0)[0]
        
        top_idx = int(np.argmax(probs))
        top_name = self.class_names[top_idx]
        top_prob = float(probs[top_idx])
        
        # Debug: print all predictions every 30 frames
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'📊 Predictions - Apple: {probs[0]:.3f}, Kiwi: {probs[1]:.3f}, Orange: {probs[2]:.3f}, Unknown: {probs[3]:.3f}')
        
        if top_prob >= self.conf:
            return top_name.lower(), top_prob  # Return lowercase for comparison
        return "unknown", top_prob

    def image_callback(self, msg):
        self.frame_count += 1
        
        # Always convert frame for visualization
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]
        
        # Calculate ROI for visualization
        roi_half = self.roi_size // 2
        center_x = w // 2
        x1 = center_x - roi_half
        x2 = center_x + roi_half
        y2 = int(h * self.roi_y_ratio)
        y1 = y2 - self.roi_size
        x1 = max(0, x1)
        x2 = min(w, x2)
        y1 = max(0, y1)
        y2 = min(h, y2)
        
        # Draw ROI rectangle (always visible)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
        
        # Add status text
        status = f"Active: {self.vision_active} | Requested: {self.requested_item}"
        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Do nothing if vision is not active or no fruit requested
        if not self.vision_active or self.requested_item is None:
            self.viz_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            return

        try:
            # Get ROI for classification
            roi, (x1, y1, x2, y2) = self.get_roi(frame)
            
            # Prepare detection message
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            # Classify the ROI
            label, confidence = self.classify_roi(roi)
            
            # Build detection message
            det = Detection2D()
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = label
            hyp.hypothesis.score = confidence
            det.results.append(hyp)
            
            cx = float((x1 + x2) / 2)
            cy = float((y1 + y2) / 2)
            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = float(x2 - x1)
            det.bbox.size_y = float(y2 - y1)
            detection_array.detections.append(det)
            
            self.det_pub.publish(detection_array)
            
            item_msg = String()
            
            # Case-insensitive comparison (both lowercase now)
            if label == self.requested_item:
                item_msg.data = 'CORRECT'
                self.item_pub.publish(item_msg)
                
                pt = PointStamped()
                pt.header = msg.header
                pt.point.x = cx
                pt.point.y = cy
                pt.point.z = 0.0
                self.pick_pub.publish(pt)
                
                self.get_logger().info(f'✅ CORRECT — {self.requested_item} at ({cx:.0f},{cy:.0f}) conf={confidence:.2f}')
            else:
                item_msg.data = 'INCORRECT'
                self.item_pub.publish(item_msg)
                self.get_logger().info(f'❌ INCORRECT — wanted: {self.requested_item} | detected: {label} (conf={confidence:.2f})')
            
            # Draw visualization with results
            color = (0, 255, 0) if label == self.requested_item else (0, 0, 255)
            text = f"{label.upper()} ({confidence:.2f})"
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
            cv2.putText(frame, text, (x1, max(25, y1 - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
            
            self.viz_pub.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))
            
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = FruitVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
