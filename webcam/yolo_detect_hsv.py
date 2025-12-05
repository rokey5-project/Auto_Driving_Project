import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np  # Numpy 추가
from ultralytics import YOLO

class DualCamYOLONode(Node):

    def __init__(self):
        super().__init__('dual_yolo_manager')
        
        self.get_logger().info("Initializing Dual Camera YOLO Manager with HSV Filter...")

        # ==========================================
        # [HSV 필터 설정]
        # ==========================================
        # 불꽃 색상 범위 (노랑~빨강)
        self.LOWER_FIRE = np.array([0, 150, 150])   # 채도(S)와 명도(V)를 약간 낮춰 잡음 제거
        self.UPPER_FIRE = np.array([35, 255, 255]) 
        self.HSV_RATIO_THRES = 0.1  # 박스 내부의 10% 이상이 불 색상이어야 인정
        # ==========================================

        self.cam1_id = 0
        self.cam2_id = 4
        
        self.pub_img1 = self.create_publisher(CompressedImage, 'cam1/yolo/compressed', 10)
        self.pub_img2 = self.create_publisher(CompressedImage, 'cam2/yolo/compressed', 10)
        self.pub_status = self.create_publisher(String, 'cam/fire_detection_status', 10)
        
        # 모델 경로 설정 (사용자 환경에 맞게 수정 필요)
        model_path = "/home/rokey/rokey_ws/src/fire_cam_detection/fire_cam_detection/best.pt"
        if not os.path.exists(model_path):
            self.get_logger().warn(f"Model not found at {model_path}. Using 'yolov8n.pt'")
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

    def is_fire_color(self, roi):
        """
        YOLO가 잡은 영역(ROI)이 실제로 불 색상인지 HSV로 검증하는 함수
        """
        if roi.size == 0: return False, 0.0
        
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_roi, self.LOWER_FIRE, self.UPPER_FIRE)
        
        fire_pixel_count = cv2.countNonZero(mask)
        total_pixels = roi.shape[0] * roi.shape[1]
        
        if total_pixels == 0: return False, 0.0

        ratio = fire_pixel_count / total_pixels
        return ratio > self.HSV_RATIO_THRES, ratio

    def timer_callback(self):
        try:
            ret1, frame1 = self.cap1.read()
            ret2, frame2 = self.cap2.read()

            if not ret1 or not ret2:
                self.get_logger().warn(f"Failed to read frames. Cam1:{ret1}, Cam2:{ret2}")
                return

            # YOLO 추론
            results = self.model.predict([frame1, frame2], imgsz=640, conf=0.4, iou=0.5, classes=0, verbose=False)

            # 결과 처리 (HSV 필터 적용)
            processed_frame1, detected1 = self.process_result(frame1, results[0])
            processed_frame2, detected2 = self.process_result(frame2, results[1])

            # 이미지 발행
            msg1 = self.bridge.cv2_to_compressed_imgmsg(processed_frame1, 'jpg')
            msg2 = self.bridge.cv2_to_compressed_imgmsg(processed_frame2, 'jpg')
            self.pub_img1.publish(msg1)
            self.pub_img2.publish(msg2)

            # 상태 메시지 생성 (HSV 필터를 통과한 결과만 반영됨)
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

            # 감지 시 로그 출력
            if detected1 or detected2:
                self.get_logger().info(f"Status: {status_msg.data}")

        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {str(e)}")

    def process_result(self, frame, result):
        is_detected_final = False # 최종 감지 여부
        
        for box in result.boxes:
            xmin, ymin, xmax, ymax = map(int, box.xyxy[0].cpu().numpy())
            conf = float(box.conf[0].cpu().numpy())
            
            # ROI 추출 (이미지 범위를 벗어나지 않게 클리핑)
            h, w, _ = frame.shape
            xmin = max(0, xmin); ymin = max(0, ymin)
            xmax = min(w, xmax); ymax = min(h, ymax)
            
            roi = frame[ymin:ymax, xmin:xmax]
            
            # [HSV 검증 수행]
            is_real_fire, ratio = self.is_fire_color(roi)
            
            if is_real_fire:
                # 진짜 불: 초록색 박스 + 감지 성공 처리
                is_detected_final = True
                color = (0, 255, 0) # Green
                label = f"Fire {conf:.2f} (HSV:{ratio:.2f})"
            else:
                # 가짜 불(색상 불일치): 파란색 박스 + 감지 실패 처리
                color = (255, 0, 0) # Blue indicates ignored
                label = f"Ignored {conf:.2f} (HSV:{ratio:.2f})"
            
            # 박스 및 라벨 그리기
            cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), color, 2)
            
            t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            # 텍스트 배경
            cv2.rectangle(frame, (xmin, ymin - t_size[1] - 5), (xmin + t_size[0], ymin), color, -1)
            # 텍스트
            cv2.putText(frame, label, (xmin, ymin - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return frame, is_detected_final

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