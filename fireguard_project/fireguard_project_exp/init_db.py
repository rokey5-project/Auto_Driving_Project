import sqlite3  # SQLite 데이터베이스 연결을 위한 모듈

# 통합 DB 파일명
DB_NAME = "fire_detection.db"  # 데이터베이스 파일 이름 설정

def create_tables():
    """DB에 필요한 테이블을 생성하는 함수"""
    try:
        # SQLite 데이터베이스에 연결 (DB 파일이 없으면 새로 생성)
        conn = sqlite3.connect(DB_NAME)
        cursor = conn.cursor()  # 데이터베이스 작업을 위한 커서 생성

        # 1) cam_detect 테이블: 현재 카메라/YOLO 상태를 저장 (마지막 상태만 유지)
        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS cam_detect (
                topic TEXT PRIMARY KEY NOT NULL,  # 토픽 이름 (기본 키로 설정)
                message TEXT,  # 메시지 (상태 정보)
                detect_time DATETIME DEFAULT CURRENT_TIMESTAMP  # 상태가 저장된 시간 (기본값: 현재 시간)
            )
            """
        )

        # 2) fire_event_log 테이블: 화재 이벤트 로그 기록 (API / ROS 등에서 공통 사용 가능)
        cursor.execute(
            """
            CREATE TABLE IF NOT EXISTS fire_event_log (
                id INTEGER PRIMARY KEY AUTOINCREMENT,  # 고유 식별자 (자동 증가)
                timestamp TEXT NOT NULL,  # 이벤트 발생 시간
                zone_id INTEGER,  # 화재 감지 구역 ID
                confidence REAL,  # 화재 감지 확신도 (신뢰도)
                webcam_id INTEGER  # 웹캠 ID (어떤 카메라에서 감지된 이벤트인지)
            )
            """
        )

        conn.commit()  # 변경사항 커밋 (DB에 반영)
        conn.close()  # 데이터베이스 연결 종료

        # 테이블 생성 완료 메시지 출력
        print(f"[*] '{DB_NAME}'에 cam_detect / fire_event_log 테이블 생성 완료")

    except Exception as e:
        # 예외가 발생하면 에러 메시지 출력
        print(f"[!] 에러 발생: {e}")

# 프로그램 실행 시, create_tables 함수 호출
if __name__ == "__main__":
    create_tables()  # 테이블 생성 함수 호출
