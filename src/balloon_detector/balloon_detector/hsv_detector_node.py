# balloon_detector/hsv_detector_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

class BalloonDetectorNode(Node):
    def __init__(self):
        super().__init__('balloon_detector')
        self.pub = self.create_publisher(Float32MultiArray, '/camera/image_deviation', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)

        self.lower = np.array([0, 130, 100])
        self.upper = np.array([10, 255, 255])

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        h, w = frame.shape[:2]
        cx = cy = dx = dy = found = area = 0.0

        if contours:
            largest = max(contours, key=cv2.contourArea)
            area_val = cv2.contourArea(largest)
            if area_val > 500:
                x, y, bw, bh = cv2.boundingRect(largest)
                cx = float(x + bw / 2)
                cy = float(y + bh / 2)
                dx = float((cx - w / 2) / (w / 2))
                dy = float(-(cy - h / 2) / (h / 2))
                found = 1.0
                area = float(area_val)
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)

        msg = Float32MultiArray()
        msg.data = [dx, dy, found, area, cx, cy]
        self.pub.publish(msg)

        cv2.imshow("Result", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BalloonDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()