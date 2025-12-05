import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sqlite3
import datetime

TOPIC_NAME = '/cam/fire_detection_status'  # 토픽 이름 정의
DB_NAME = 'fire_detection.db'  # SQLite DB 파일 이름

class DBRecorder(Node):
    def __init__(self):
        super().__init__('db_recorder_node')  # 부모 클래스(Node) 초기화, 노드 이름 'db_recorder_node'
        
        # SQLite DB 연결 및 커서 생성
        self.conn = sqlite3.connect(DB_NAME)
        self.cursor = self.conn.cursor()
        
        self.FRAME_THRESHOLD = 3  # 연속된 동일 메시지가 몇 번 수신되어야 DB에 저장할지 결정하는 임계값
        self.consecutive_count = 0    # 연속된 동일 메시지 수
        self.candidate_data = None    # 현재 수신된 메시지(후보값)
        self.last_saved_data = None  # DB에 저장된 마지막 메시지
        
        # 토픽 구독: '/cam/fire_detection_status' 토픽에 대해 String 메시지 타입을 구독
        self.subscription = self.create_subscription(
            String,  # 메시지 타입
            TOPIC_NAME,  # 구독할 토픽
            self.listener_callback,  # 메시지 수신 시 호출될 콜백 함수
            10  # 큐 사이즈: 한 번에 10개 메시지를 수신
        )
        self.get_logger().info(f'[{TOPIC_NAME}] 토픽 구독 및 DB 저장을 시작합니다...')  # 로깅

    def listener_callback(self, msg):
        """ 메시지를 수신했을 때 호출되는 콜백 함수 """
        try:
            current_data = str(msg.data)  # 수신된 메시지 데이터
            
            # 메시지가 이전과 동일하면 카운트를 증가시키고, 다르면 카운트를 리셋
            if current_data == self.candidate_data:
                self.consecutive_count += 1
            else:
                self.candidate_data = current_data
                self.consecutive_count = 0

            # 연속된 동일 메시지가 일정 횟수 이상이면 DB에 저장
            if self.consecutive_count >= self.FRAME_THRESHOLD:
                if self.candidate_data != self.last_saved_data:  # 이전에 저장된 데이터와 다르면 저장
                    self.save_to_db(self.candidate_data)
                    self.last_saved_data = self.candidate_data  # 마지막 저장된 데이터 업데이트
                    self.get_logger().info(f'상태 변경: {self.last_saved_data}')  # 상태 변경 로그 출력

        except Exception as e:
            self.get_logger().error(f'에러 발생: {e}')  # 예외 발생 시 로그 출력

    def save_to_db(self, data):
        """ DB에 데이터를 저장하는 함수 """
        try:
            # INSERT OR REPLACE 쿼리: 동일한 토픽과 메시지가 있으면 덮어쓰고, 없으면 새로 추가
            self.cursor.execute(
                "INSERT OR REPLACE INTO cam_detect (topic, message, detect_time) VALUES (?, ?, CURRENT_TIMESTAMP)",
                (TOPIC_NAME, data)  # 토픽 이름과 데이터, 현재 시간
            )
            self.conn.commit()  # 변경사항 커밋
        except Exception as e:
            self.get_logger().error(f'DB 쿼리 실패: {e}')  # DB 쿼리 실패 시 로그 출력

    def __del__(self):
        """ 객체가 파괴될 때 DB 연결 종료 """
        if hasattr(self, 'conn'):
            self.conn.close()  # DB 연결 종료

def main(args=None):
    """ main 함수: 노드 실행 """
    rclpy.init(args=args)  # ROS2 초기화
    
    db_recorder = DBRecorder()  # DBRecorder 객체 생성

    try:
        rclpy.spin(db_recorder)  # 메시지를 수신할 때까지 대기
    except KeyboardInterrupt:
        pass  # 사용자 인터럽트 시 종료
    finally:
        db_recorder.destroy_node()  # 노드 종료
        rclpy.shutdown()  # ROS2 종료

if __name__ == '__main__':
    main()  # main 함수 실행
