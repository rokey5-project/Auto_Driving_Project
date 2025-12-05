from typing import Any, Dict

shared_state: Dict[str, Any] = {

    "fire_event": {
        "detected": False,
        "confidence": 0.0,
        "zone_id": None,
        "last_fire_time": 0.0,
    },

    "depth": {
        "robot4": None,
        "robot6": None,
    },

    "battery": {
        "robot4": None,
        "robot6": None,
    },

    "camera": {
        "robot4": False,
        "robot6": False,
        "cam1": False,
        "cam2": False,
    },

    "latest_frame": {
        "robot4": None,    # numpy BGR
        "robot6": None,
        "cam1": None,
        "cam2": None,
    },

    "audio": {
        "target_gain": 72,
        "should_beep": False
    },

    "robot_pose": {
        "robot4": {
            "x": None,
            "y": None,
            "yaw": None
        },
        "robot6": {
            "x": None,
            "y": None,
            "yaw": None
        }
    },

    # ============================
    # ▼ SLAM MAP 로봇 경로 저장 추가 (기존 기능 절대 수정 X)
    # ============================
    "pose_history_robot4": [],
    "pose_history_robot6": [],
}
