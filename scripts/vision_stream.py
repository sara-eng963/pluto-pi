#!/usr/bin/env python3
"""
vision_stream.py
Subscribes to /image_raw and serves an MJPEG web stream.
Open http://172.20.10.2:8080 in your browser to watch live.
"""

import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response

HOST = '172.20.10.2'
PORT = 8080

app = Flask(__name__)

# Shared latest JPEG frame (bytes), protected by a lock
_frame_lock = threading.Lock()
_latest_frame: bytes | None = None


def set_frame(frame_bytes: bytes):
    global _latest_frame
    with _frame_lock:
        _latest_frame = frame_bytes


def get_frame() -> bytes | None:
    with _frame_lock:
        return _latest_frame


# ── MJPEG generator ────────────────────────────────────────────────────────────
def generate():
    while True:
        frame = get_frame()
        if frame is None:
            continue
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n'
        )


@app.route('/')
def index():
    return (
        '<html><body style="background:#000;margin:0">'
        '<img src="/stream" style="width:100%;height:auto">'
        '</body></html>'
    )


@app.route('/stream')
def stream():
    return Response(
        generate(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


# ── ROS2 node ──────────────────────────────────────────────────────────────────
class VisionStreamNode(Node):
    def __init__(self):
        super().__init__('vision_stream_node')
        self.bridge = CvBridge()
        self.create_subscription(Image, '/image_raw', self._image_cb, 10)
        self.get_logger().info(f'📡 Streaming at http://{HOST}:{PORT}')

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            set_frame(buf.tobytes())


def main(args=None):
    rclpy.init(args=args)
    node = VisionStreamNode()

    # Run Flask in a background daemon thread
    flask_thread = threading.Thread(
        target=lambda: app.run(host=HOST, port=PORT, threaded=True),
        daemon=True
    )
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
