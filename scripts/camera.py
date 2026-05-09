#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class StableCamera(Node):
    def __init__(self):
        super().__init__('stable_camera')
        self.pub = self.create_publisher(Image, '/image_raw', 10)
        self.bridge = CvBridge()
        
        # Try to open camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera!')
            sys.exit(1)
        
        # Set resolution
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.get_logger().info('✅ Camera publishing to /image_raw')
        self.timer = self.create_timer(0.066, self.timer_cb)
        self.frame_count = 0
    
    def timer_cb(self):
        ret, frame = self.cap.read()
        if ret:
            self.frame_count += 1
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')

def main(args=None):
    rclpy.init(args=args)
    node = StableCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
