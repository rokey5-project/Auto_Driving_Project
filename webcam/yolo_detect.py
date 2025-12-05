import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class DualCamYOLONode(Node):

    def __init__(self):
        super().__init__('dual_yolo_manager')
        
        self.get_logger().info("Initializing Dual Camera YOLO Manager...")

        self.cam1_id = 0
        self.cam2_id = 4
        
        self.pub_img1 = self.create_publisher(CompressedImage, 'cam1/yolo/compressed', 10)
        self.pub_img2 = self.create_publisher(CompressedImage, 'cam2/yolo/compressed', 10)
        self.pub_status = self.create_publisher(String, 'cam/fire_detection_status', 10)
        
        model_path = "/home/rokey/rokey_ws/src/fire_cam_detection/fire_cam_detection/best.pt"
        if not os.path.exists(model_path):
            self.get_logger().warn(f"Model not found. Using 'yolov8n.pt'")
            model_path = "yolov8n.pt"
            
        self.get_logger().info(f"Loading YOLO model... ({model_path})")
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        self.cap1 = self.init_camera(self.cam1_id)
        self.cap2 = self.init_camera(self.cam2_id)

        if not self.cap1.isOpened() or not self.cap2.isOpened():
            self.get_logger().error("One of the webcams couldn't be opened!")

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.get_logger().info("Dual Camera System Initialized.")

    def init_camera(self, cam_id):
        cap = cv2.VideoCapture(cam_id)
        return cap

    def timer_callback(self):
        try:
            ret1, frame1 = self.cap1.read()
            ret2, frame2 = self.cap2.read()

            if not ret1 or not ret2:
                self.get_logger().warn(f"Failed to read frames.{ret1}, {ret2}")
                return

            results = self.model.predict([frame1, frame2], imgsz=640, conf=0.5, iou=0.5, classes=0, verbose=False)

            processed_frame1, detected1 = self.process_result(frame1, results[0])
            processed_frame2, detected2 = self.process_result(frame2, results[1])

            msg1 = self.bridge.cv2_to_compressed_imgmsg(processed_frame1, 'jpg')
            msg2 = self.bridge.cv2_to_compressed_imgmsg(processed_frame2, 'jpg')
            self.pub_img1.publish(msg1)
            self.pub_img2.publish(msg2)

            status_msg = String()
            if detected1 and detected2:
                status_msg.data = "Both Detected"
            elif detected1:
                status_msg.data = "Cam1 Detected"
            elif detected2:
                status_msg.data = "Cam2 Detected"
            else:
                status_msg.data = "No Detected"
            
            self.pub_status.publish(status_msg)

            if detected1 or detected2:
                self.get_logger().info(f"Status: {status_msg.data}")

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")

    def process_result(self, frame, result):
        is_detected = False
        
        for box in result.boxes:
            cls_id = int(box.cls[0].cpu().numpy())
            
            is_detected = True
            
            xmin, ymin, xmax, ymax = box.xyxy[0].cpu().numpy()
            
            conf = float(box.conf[0].cpu().numpy())
            
            label = f"fire {conf:.2f}"
            
            cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 0, 255), 2)
            
            t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            cv2.rectangle(frame, (int(xmin), int(ymin) - t_size[1] - 5), (int(xmin) + t_size[0], int(ymin)), (0, 0, 255), -1)
            cv2.putText(frame, label, (int(xmin), int(ymin) - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return frame, is_detected

    def cleanup(self):
        if self.cap1.isOpened(): self.cap1.release()
        if self.cap2.isOpened(): self.cap2.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = DualCamYOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
