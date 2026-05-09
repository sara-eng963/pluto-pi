#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CommNode(Node):
    def __init__(self):
        super().__init__('pi_comm_node')
        
        # ===== SUBSCRIBER: receives from ESP32 =====
        self.subscription = self.create_subscription(
            String,
            'esp_comm_test',      # ESP32 publishes here
            self.esp_callback,
            10)
        
        # ===== PUBLISHER: sends to ESP32 =====
        self.publisher = self.create_publisher(String, 'pi_comm_test', 10)
        
        # Timer to publish every second
        self.timer = self.create_timer(1.0, self.publish_callback)
        self.counter = 0
        
        self.get_logger().info('Pi Node Ready!')
        self.get_logger().info('  Publishing to: pi_comm_test (ESP32 subscribes)')
        self.get_logger().info('  Subscribing to: esp_comm_test (ESP32 publishes)')
    
    def esp_callback(self, msg):
        # This receives messages from ESP32
        self.get_logger().info(f'📩 FROM ESP32: "{msg.data}"')
    
    def publish_callback(self):
        # This sends messages to ESP32
        msg = String()
        msg.data = f'hello esp {self.counter}'
        self.publisher.publish(msg)
        self.get_logger().info(f'📤 TO ESP32: "{msg.data}"')
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
