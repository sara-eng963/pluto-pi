#!/usr/bin/env python3

import argparse
import socket
import time
from pathlib import Path

import cv2


DEFAULT_CAMERA_DEVICE = "/dev/video0"
DEFAULT_IMAGE_COUNT = 100
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_FPS = 30


def parse_args():
    parser = argparse.ArgumentParser(
        description="Capture a batch of photos from the camera for easy SCP transfer."
    )
    parser.add_argument(
        "-n",
        "--count",
        type=int,
        default=DEFAULT_IMAGE_COUNT,
        help=f"number of images to capture, default: {DEFAULT_IMAGE_COUNT}",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        default=None,
        help="folder to save images into, default: captured_images/YYYYmmdd_HHMMSS",
    )
    parser.add_argument(
        "--device",
        default=DEFAULT_CAMERA_DEVICE,
        help=f"camera device path, default: {DEFAULT_CAMERA_DEVICE}",
    )
    parser.add_argument(
        "--delay",
        type=float,
        default=0.2,
        help="delay in seconds between saved images, default: 0.2",
    )
    parser.add_argument(
        "--warmup-frames",
        type=int,
        default=10,
        help="frames to discard before saving, default: 10",
    )
    return parser.parse_args()


def open_camera(device):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, DEFAULT_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, DEFAULT_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, DEFAULT_FPS)
    return cap


def main():
    args = parse_args()

    if args.count <= 0:
        raise SystemExit("Image count must be greater than 0.")
    if args.delay < 0:
        raise SystemExit("Delay must be 0 or greater.")
    if args.warmup_frames < 0:
        raise SystemExit("Warmup frames must be 0 or greater.")

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    output_dir = args.output_dir or Path("captured_images") / timestamp
    output_dir.mkdir(parents=True, exist_ok=True)

    cap = open_camera(args.device)
    if not cap.isOpened():
        raise SystemExit(f"Camera failed to open: {args.device}")

    print(f"Camera opened: {args.device}")
    print(f"Saving {args.count} images to: {output_dir.resolve()}")

    try:
        for _ in range(args.warmup_frames):
            cap.read()

        saved = 0
        while saved < args.count:
            ret, frame = cap.read()
            if not ret:
                print("Frame read failed, retrying...")
                time.sleep(0.1)
                continue

            saved += 1
            image_path = output_dir / f"image_{saved:03d}.jpg"
            if not cv2.imwrite(str(image_path), frame):
                raise SystemExit(f"Failed to save image: {image_path}")

            print(f"Saved {saved:03d}/{args.count}: {image_path}")
            if saved < args.count and args.delay:
                time.sleep(args.delay)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        cap.release()
        print("Camera released.")

    host = socket.gethostname()
    print("\nTo copy these images from your laptop, run something like:")
    print(f"scp -r pi@{host}:{output_dir.resolve()} .")


if __name__ == "__main__":
    main()
