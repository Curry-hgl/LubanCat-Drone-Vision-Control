import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge
from rknnlite.api import RKNNLite

class RKNNBalloonDetector(Node):
    def __init__(self):
        super().__init__('rknn_balloon_detector')
        
        self.declare_parameter('device_path', '/dev/video0')
        self.declare_parameter('model_path', 'yolo11n.rknn')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('target_class', -1)
        self.declare_parameter('video_output', False)
        
        device = self.get_parameter('device_path').value
        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('conf_threshold').value
        self.target_class = self.get_parameter('target_class').value
        self.video_output = self.get_parameter('video_output').value
        
        # 加载 RKNN
        self.rknn = RKNNLite()
        ret = self.rknn.load_rknn(model_path)
        if ret != 0:
            self.get_logger().error(f'加载 RKNN 失败: {ret}')
            return
        ret = self.rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)
        if ret != 0:
            self.get_logger().error(f'初始化 NPU 失败: {ret}')
            return
        self.get_logger().info(f'✅ RKNN 模型加载完成: {model_path}')
        
        self.names = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane',
            5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
            10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench',
            14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow',
            20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack',
            25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee',
            30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat',
            35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket',
            39: 'bottle', 40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife',
            44: 'spoon', 45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich',
            49: 'orange', 50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza',
            54: 'donut', 55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant',
            59: 'bed', 60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop',
            64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave',
            69: 'oven', 70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book',
            74: 'clock', 75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier',
            79: 'toothbrush'}
        
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.bridge = CvBridge()
        
        self.publisher = self.create_publisher(Float32MultiArray, '/camera/image_deviation', 10)
        self.timer = self.create_timer(0.033, self.process_frame)
        self.get_logger().info('✅ RKNN 检测节点已启动')
    
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('摄像头读取失败')
            return
        
        h, w = frame.shape[:2]
        dx, dy, found, area = 0.0, 0.0, 0.0, 0.0
        
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_resized = cv2.resize(img, (640, 640))
        img_input = np.expand_dims(img_resized, 0).astype(np.float32)
        
        outputs = self.rknn.inference(inputs=[img_input])
        
        output = outputs[0]
        pred = np.squeeze(output).T
        
        scores = np.max(pred[:, 4:], axis=1)
        mask = scores > self.conf_thresh
        pred = pred[mask]
        scores = scores[mask]
        
        if len(pred) > 0:
            classes = np.argmax(pred[:, 4:], axis=1)
            boxes = pred[:, :4]
            
            x, y, bw, bh = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
            x1 = (x - bw/2) * w / 640
            y1 = (y - bh/2) * h / 640
            x2 = (x + bw/2) * w / 640
            y2 = (y + bh/2) * h / 640
            
            if self.target_class >= 0:
                cls_mask = (classes == self.target_class)
                if not np.any(cls_mask):
                    self.publish_msg(dx, dy, 0.0, 0.0)
                    if self.video_output:
                        cv2.imshow('RKNN Detection', frame)
                        cv2.waitKey(1)
                    return
                x1, y1, x2, y2 = x1[cls_mask], y1[cls_mask], x2[cls_mask], y2[cls_mask]
                scores = scores[cls_mask]
                classes = classes[cls_mask]
            
            boxes_for_nms = np.column_stack((x1, y1, x2-x1, y2-y1)).astype(np.float32)
            indices = cv2.dnn.NMSBoxes(boxes_for_nms.tolist(), scores.tolist(), self.conf_thresh, 0.45)
            
            if len(indices) > 0:
                # 兼容不同 OpenCV 版本
                if isinstance(indices, tuple):
                    idx = int(indices[0])
                elif isinstance(indices, np.ndarray):
                    idx = int(indices.flatten()[0])
                else:
                    idx = int(indices[0])

                bx1, by1, bx2, by2 = int(x1[idx]), int(y1[idx]), int(x2[idx]), int(y2[idx])
                cx = int((bx1 + bx2) / 2)
                cy = int((by1 + by2) / 2)
                area = (bx2 - bx1) * (by2 - by1)
                
                dx = (cx - w/2) / (w/2)
                dy = -(cy - h/2) / (h/2)
                found = 1.0
                
                if self.video_output:
                    cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    label = f"{self.names.get(int(classes[idx]), 'unknown')} {scores[idx]:.2f}"
                    cv2.putText(frame, label, (bx1, by1-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.putText(frame, f"dx: {dx:.2f} dy: {dy:.2f}", (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        self.publish_msg(dx, dy, found, area)
        
        if self.video_output:
            cv2.imshow('RKNN Detection', frame)
            cv2.waitKey(1)
    
    def publish_msg(self, dx, dy, found, area):
        msg = Float32MultiArray()
        msg.data = [float(dx), float(dy), float(found), float(area)]
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RKNNBalloonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()