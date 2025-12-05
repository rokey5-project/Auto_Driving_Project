import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from .utils import shared_state

# ============================================================
# DEPTH HANDLER NODE (깊이 이미지 처리 노드)
# ============================================================
class DepthHandler(Node):
    def __init__(self):
        """DepthHandler 노드 초기화"""
        super().__init__("depth_handler")

        self.bridge = CvBridge()  # CvBridge 초기화 (ROS 이미지를 OpenCV 형식으로 변환)

        # ============================================================
        # DEPTH TOPIC SUBSCRIPTIONS (깊이 토픽 구독)
        # ============================================================
        self.sub_depth_A = self.create_subscription(
            Image,
            "/robot4/oakd/stereo/image_raw/compressedDepth",  # 카메라 1
            self.cb_depth_A,
            10,
        )
        self.sub_depth_B = self.create_subscription(
            Image,
            "/robot6/oakd/stereo/image_raw/compressedDepth",  # 카메라 2
            self.cb_depth_B,
            10,
        )

    # ============================================================
    # DEPTH CALCULATION FUNCTION (깊이 계산 함수)
    # ============================================================
    def calc_distance(self, depth_img):
        """
        깊이 이미지를 사용하여 물체까지의 거리를 계산합니다.
        depth_img: numpy array (깊이 이미지)
        """
        # 1) 0 제거 (센서 오류)
        depth_filtered = depth_img[depth_img > 0]

        if depth_filtered.size == 0:
            return None

        # 2) 중앙 100×100 ROI 사용하는 게 더 안정적
        h, w = depth_img.shape
        cx, cy = w//2, h//2

        roi = depth_img[cy-50:cy+50, cx-50:cx+50]
        roi_filtered = roi[roi > 0]

        if roi_filtered.size == 0:
            min_depth = np.min(depth_filtered)
        else:
            min_depth = np.min(roi_filtered)

        # Depth가 mm 단위라면 m 단위로 변환
        if min_depth > 50:   # 보통 mm 값이므로
            return min_depth / 1000.0

        # 이미 m 단위면 그대로 반환
        return float(min_depth)

    # ============================================================
    # CALLBACK FUNCTION A (카메라 1의 Depth 수신 처리)
    # ============================================================
    def cb_depth_A(self, msg):
        """카메라 1에서 수신한 깊이 이미지를 처리하는 콜백 함수"""
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            distance_m = self.calc_distance(depth_img)  # 깊이 계산
            shared_state["depth"]["A"] = distance_m  # 공유 상태에 저장
        except Exception as e:
            self.get_logger().error(f"Depth A 처리 오류: {e}")

    # ============================================================
    # CALLBACK FUNCTION B (카메라 2의 Depth 수신 처리)
    # ============================================================
    def cb_depth_B(self, msg):
        """카메라 2에서 수신한 깊이 이미지를 처리하는 콜백 함수"""
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            distance_m = self.calc_distance(depth_img)  # 깊이 계산
            shared_state["depth"]["B"] = distance_m  # 공유 상태에 저장
        except Exception as e:
            self.get_logger().error(f"Depth B 처리 오류: {e}")

# ============================================================
# MAIN FUNCTION (메인 함수)
# ============================================================
def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 실행"""
    rclpy.init(args=args)
    node = DepthHandler()
    try:
        rclpy.spin(node)  # 노드 실행
    finally:
        node.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS2 종료

if __name__ == "__main__":
    main()
