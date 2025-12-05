import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from .utils import shared_state

# ============================================================
# CAMERA HANDLER NODE (카메라 상태 처리 노드)
# ============================================================
class CameraHandler(Node):
    def __init__(self):
        """CameraHandler 노드 초기화"""
        super().__init__("camera_handler")
        self.bridge = CvBridge()  # CvBridge 초기화 (ROS 이미지를 OpenCV 형식으로 변환)

        # ============================================================
        # 카메라 RGB 영상 수신 (압축되지 않은 이미지)
        # ============================================================
        self.create_subscription(
            Image,
            "/robot4/oakd/rgb/image_raw",  # 카메라 1
            self.cb_cam_A, 10
        )
        self.create_subscription(
            Image,
            "/robot6/oakd/rgb/image_raw",  # 카메라 2
            self.cb_cam_B, 10
        )

        # ============================================================
        # YOLO 관련 카메라 수신 (압축된 이미지)
        # ============================================================
        self.create_subscription(
            CompressedImage,
            "/cam1/yolo/compressed",  # 카메라 1 YOLO 출력
            self.cb_cam1, 10
        )
        self.create_subscription(
            CompressedImage,
            "/cam2/yolo/compressed",  # 카메라 2 YOLO 출력
            self.cb_cam2, 10
        )

    # ============================================================
    # 카메라 1 (RGB 이미지 수신)
    # ============================================================
    def cb_cam_A(self, msg):
        """카메라 1의 RGB 이미지를 처리하는 콜백 함수"""
        shared_state["camera"]["A"] = True
        # 압축되지 않은 RGB 이미지를 NumPy 배열로 변환
        shared_state["latest_frame"]["A"] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # ============================================================
    # 카메라 2 (RGB 이미지 수신)
    # ============================================================
    def cb_cam_B(self, msg):
        """카메라 2의 RGB 이미지를 처리하는 콜백 함수"""
        shared_state["camera"]["B"] = True
        # 압축되지 않은 RGB 이미지를 NumPy 배열로 변환
        shared_state["latest_frame"]["B"] = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # ============================================================
    # 카메라 1 (YOLO 이미지 수신 - 압축된 이미지)
    # ============================================================
    def cb_cam1(self, msg):
        """카메라 1의 YOLO 이미지를 처리하는 콜백 함수"""
        shared_state["camera"]["cam1"] = True
        # 압축된 YOLO 이미지를 NumPy 배열로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)  # YOLO 바이트 데이터를 NumPy 배열로 변환
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 압축된 이미지를 디코딩
        shared_state["latest_frame"]["cam1"] = img  # 디코딩된 이미지를 저장

    # ============================================================
    # 카메라 2 (YOLO 이미지 수신 - 압축된 이미지)
    # ============================================================
    def cb_cam2(self, msg):
        """카메라 2의 YOLO 이미지를 처리하는 콜백 함수"""
        shared_state["camera"]["cam2"] = True
        # 압축된 YOLO 이미지를 NumPy 배열로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)  # YOLO 바이트 데이터를 NumPy 배열로 변환
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # 압축된 이미지를 디코딩
        shared_state["latest_frame"]["cam2"] = img  # 디코딩된 이미지를 저장

# ============================================================
# MAIN FUNCTION (메인 함수)
# ============================================================
def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 실행"""
    rclpy.init(args=args)
    node = CameraHandler()
    try:
        rclpy.spin(node)  # 노드 실행
    finally:
        node.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS2 종료

if __name__ == "__main__":
    main()
