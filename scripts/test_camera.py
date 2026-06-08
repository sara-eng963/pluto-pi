import cv2

CAMERA_DEVICE = "/dev/video0"

cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

print("Camera opened:", cap.isOpened())

ret, frame = cap.read()

print("Frame read:", ret)

if ret:
    print("Frame shape:", frame.shape)
    cv2.imwrite("test_frame.jpg", frame)
    print("Saved image: test_frame.jpg")

cap.release()