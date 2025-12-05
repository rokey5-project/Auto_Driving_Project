import time
import sqlite3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .utils import shared_state

DB_PATH = "/home/rokey/fireguard_project_main/fire_detection.db"  # DB 경로
TOPIC_NAME = "/cam/fire_detection_status"  # 구독할 화재 감지 상태 토픽

# ============================================================
# FIRE EVENT HANDLER NODE (화재 이벤트 처리 노드)
# ============================================================
class FireEventHandler(Node):

    def __init__(self):
        """FireEventHandler 노드 초기화"""
        super().__init__("fire_event_handler")

        # ============================================================
        # DATABASE CONNECTION (DB 연결)
        # ============================================================
        self.conn = sqlite3.connect(DB_PATH, check_same_thread=False, isolation_level=None)
        self.cursor = self.conn.cursor()
        self._ensure_tables()  # 테이블이 없으면 생성

        # ============================================================
        # FIRE EVENT PARAMETER (화재 이벤트 파라미터 설정)
        # ============================================================
        self.FRAME_THRESHOLD = 3  # 연속된 값이 같은지 체크하는 임계값
        self.consecutive_count = 0  # 연속된 카운트
        self.candidate_data = None  # 후보 데이터 (최근 값)
        self.last_saved_data = None  # 마지막으로 저장된 데이터

        # ============================================================
        # SUBSCRIPTION (화재 감지 상태 토픽 구독)
        # ============================================================
        self.sub = self.create_subscription(
            String,
            TOPIC_NAME,
            self.cb_fire_status,
            10,
        )

        self.get_logger().info(
            f"🔥 FireEventHandler 활성화됨 ({TOPIC_NAME} 구독 중, DB={DB_PATH})"
        )

    # ============================================================
    # DATABASE TABLE CREATION (DB 테이블 생성)
    # ============================================================
    def _ensure_tables(self):
        """필요한 테이블이 없으면 생성하는 함수"""
        self.cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS cam_detect (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                topic TEXT NOT NULL,
                message TEXT,
                detect_time DATETIME DEFAULT CURRENT_TIMESTAMP
            );
            """
        )

        self.cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS fire_event_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                zone_id INTEGER DEFAULT 0,
                confidence REAL,
                webcam_id INTEGER
            );
            """
        )

    # ============================================================
    # CALLBACK FUNCTION (화재 상태 메시지 처리)
    # ============================================================
    def cb_fire_status(self, msg: String):
        """화재 상태를 처리하는 콜백 함수"""
        raw = msg.data.strip()  # 수신된 메시지에서 공백 제거

        try:
            conf = float(raw)  # confidence 값을 float로 변환
        except:
            self.get_logger().error(f"🔥 변환 실패: {raw}")
            return

        # 화재 발생 여부 판단: "No Detected"가 아니면 화재 발생으로 판단
        detected = raw != "No Detected"

        # 화재 상태를 shared_state에 저장
        fire_state = shared_state["fire_event"]
        fire_state["detected"] = detected
        fire_state["confidence"] = conf
        if detected:
            fire_state["last_fire_time"] = time.time()

        # 화재 감지 조건: "No Detected"가 아니거나, confidence가 0.7 이상
        if detected or conf >= 0.7:
            fire_state["detected"] = True
        else:
            fire_state["detected"] = False

        # 연속된 데이터 처리
        current_data = f"{conf:.4f}"

        if current_data == self.candidate_data:
            self.consecutive_count += 1
        else:
            self.candidate_data = current_data
            self.consecutive_count = 0

        # 3번 연속 동일한 값이 들어오면 DB에 기록
        if self.consecutive_count >= self.FRAME_THRESHOLD:
            if self.candidate_data != self.last_saved_data:
                self._save_to_db(conf, fire_state["detected"])
                self.last_saved_data = self.candidate_data
                self.get_logger().info(f"🔥 DB 기록됨 {conf:.4f}")

    # ============================================================
    # SAVE TO DATABASE (DB에 데이터 저장)
    # ============================================================
    def _save_to_db(self, conf: float, detected: bool):
        """화재 데이터를 DB에 저장하는 함수"""
        try:
            # cam_detect 테이블에 화재 상태 기록
            self.cursor.execute(
                """
                INSERT INTO cam_detect (topic, message, detect_time)
                VALUES (?, ?, CURRENT_TIMESTAMP)
                """,
                (TOPIC_NAME, f"{conf:.4f}")
            )

            # fire_event_log 테이블에 화재 이벤트 기록
            webcam_id = self.get_webcam_id()  # 웹캠 ID 가져오기

            self.cursor.execute(
                """
                INSERT INTO fire_event_log (timestamp, zone_id, confidence, webcam_id)
                VALUES (CURRENT_TIMESTAMP, 0, ?, ?)
                """,
                (conf, webcam_id)
            )

        except Exception as e:
            self.get_logger().error(f"[DB ERROR] {e}")

    # ============================================================
    # GET WEBCAM ID (웹캠 ID 반환)
    # ============================================================
    def get_webcam_id(self):
        """웹캠 ID를 반환하는 함수 (연결된 웹캠의 ID 확인)"""
        if shared_state["camera"].get("cam1"):
            return 1  # 웹캠 1이 연결되어 있으면 1 반환
        elif shared_state["camera"].get("cam2"):
            return 2  # 웹캠 2가 연결되어 있으면 2 반환
        else:
            return None  # 웹캠 연결 안됨

    # ============================================================
    # NODE DESTRUCTION (노드 종료 시 DB 연결 종료)
    # ============================================================
    def destroy_node(self):
        """노드 종료 시 DB 연결을 닫는 함수"""
        try:
            self.conn.close()  # DB 연결 종료
        except:
            pass
        super().destroy_node()  # 노드 종료

# ============================================================
# MAIN FUNCTION (메인 함수)
# ============================================================
def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 실행"""
    rclpy.init(args=args)
    node = FireEventHandler()
    try:
        rclpy.spin(node)  # 노드 실행
    finally:
        node.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS2 종료

if __name__ == "__main__":
    main()
