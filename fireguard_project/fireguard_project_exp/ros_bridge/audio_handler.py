import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Empty
from .utils import shared_state

# ============================================================
# AUDIO HANDLER NODE (오디오 명령 처리 노드)
# ============================================================
class AudioHandler(Node):
    def __init__(self):
        """AudioHandler 노드 초기화"""
        super().__init__('audio_handler')

        # ============================================================
        # PUBLISHERS (오디오 관련 토픽 퍼블리셔)
        # ============================================================
        self.pub_gain = self.create_publisher(Int32, "/robot4/audio_gain", 10)  # 오디오 게인 퍼블리셔
        self.pub_beep = self.create_publisher(Empty, "/robot4/audio_beep", 10)  # 비프 사운드 퍼블리셔

        # ============================================================
        # TIMER (오디오 명령 체크 타이머)
        # ============================================================
        self.timer = self.create_timer(0.2, self.check_audio_commands)  # 주기적으로 오디오 명령 체크

        # ============================================================
        # LAST GAIN (이전에 설정된 오디오 게인 값)
        # ============================================================
        self.last_gain = shared_state["audio"]["target_gain"]

        self.get_logger().info("AudioHandler 활성화됨")

    def check_audio_commands(self):
        """주기적으로 오디오 명령을 확인하고 처리하는 함수"""
        audio_state = shared_state["audio"]
        target_gain = audio_state["target_gain"]

        # ============================================================
        # AUDIO GAIN (오디오 게인 변경 확인)
        # ============================================================
        if target_gain != self.last_gain:
            msg = Int32()  # Int32 메시지 생성
            msg.data = int(target_gain)
            self.pub_gain.publish(msg)  # 오디오 게인 퍼블리시
            self.last_gain = target_gain  # 이전 게인 값 업데이트

        # ============================================================
        # BEEP COMMAND (비프 사운드 명령 처리)
        # ============================================================
        if audio_state["should_beep"]:
            self.pub_beep.publish(Empty())  # 비프 사운드 퍼블리시
            audio_state["should_beep"] = False  # 비프 사운드 처리 후 플래그 초기화

# ============================================================
# MAIN FUNCTION (메인 함수)
# ============================================================
def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 실행"""
    rclpy.init(args=args)
    node = AudioHandler()
    try:
        rclpy.spin(node)  # 노드 실행
    finally:
        node.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()
