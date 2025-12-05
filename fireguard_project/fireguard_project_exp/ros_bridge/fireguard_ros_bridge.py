import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import rclpy
from rclpy.node import Node
import math
import time
import sqlite3
import numpy as np
import cv2
import requests

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState, Image, CompressedImage
from std_msgs.msg import String, Int32, Empty, Bool

from irobot_create_msgs.msg import DockStatus  # 실제 Dock 타입

from rclpy.qos import qos_profile_sensor_data

from ros_bridge.utils import shared_state

DB_PATH = "/home/rokey/fireguard_project_exp/fire_detection.db"


class FireGuardBridge(Node):
    def __init__(self):
        super().__init__("fireguard_ros_bridge")

        # ============================
        # Camera
        # ============================
        from cv_bridge import CvBridge
        self.bridge = CvBridge()

        self.create_subscription(
            Image,
            "/robot4/oakd/rgb/image_raw",
            self.cb_cam_robot4,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            "/robot6/oakd/rgb/image_raw",
            self.cb_cam_robot6,
            qos_profile_sensor_data
        )

        self.create_subscription(
            CompressedImage,
            "/cam1/yolo/compressed",
            self.cb_yolo1,
            qos_profile_sensor_data
        )

        self.create_subscription(
            CompressedImage,
            "/cam2/yolo/compressed",
            self.cb_yolo2,
            qos_profile_sensor_data
        )

        # ============================
        # Depth
        # ============================
        self.create_subscription(
            Image,
            "/robot4/oakd/stereo/image_raw",
            self.cb_depth_robot4,
            qos_profile_sensor_data
        )

        self.create_subscription(
            Image,
            "/robot6/oakd/stereo/image_raw",
            self.cb_depth_robot6,
            qos_profile_sensor_data
        )

        # ============================
        # Battery
        # ============================
        self.last_bat_4 = 0.0
        self.last_bat_6 = 0.0
        self.bat_interval = 60.0

        self.create_subscription(
            BatteryState,
            "/robot4/battery_state",
            self.cb_battery_robot4,
            10
        )

        self.create_subscription(
            BatteryState,
            "/robot6/battery_state",
            self.cb_battery_robot6,
            10
        )

        # ============================
        # Pose
        # ============================
        self.last_pose4 = 0.0
        self.last_pose6 = 0.0

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/robot4/amcl_pose",
            self.cb_pose4,
            qos_profile_sensor_data
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/robot6/amcl_pose",
            self.cb_pose6,
            qos_profile_sensor_data
        )
        # QoS 10(기본)로 한 번 더 구독 추가
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/robot4/amcl_pose",
            self.cb_pose4,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            "/robot6/amcl_pose",
            self.cb_pose6,
            10
        )
        # ============================
        # Audio
        # ============================
        self.pub_gain = self.create_publisher(
            Int32, "/robot4/audio_gain", 10
        )
        self.pub_beep = self.create_publisher(
            Empty, "/robot4/audio_beep", 10
        )
        self.create_timer(0.2, self.timer_audio)

        # ============================
        # DB
        # ============================
        self.conn = sqlite3.connect(DB_PATH, check_same_thread=False)
        self.cursor = self.conn.cursor()
        self.ensure_tables()

        self.create_subscription(
            String,
            "/cam/fire_detection_status",
            self.cb_fire_event,
            10
        )

        # ============================
        # DOCK STATUS
        # ============================
        self.last_dock_state = {
            "robot4": None,
            "robot6": None
        }

        # 실제 DockStatus 타입
        self.create_subscription(
            DockStatus,
            "/robot4/dock_status",
            lambda msg: self.cb_dock_status_real(msg, "robot4"),
            10
        )

        self.create_subscription(
            DockStatus,
            "/robot6/dock_status",
            lambda msg: self.cb_dock_status_real(msg, "robot6"),
            10
        )

        self.get_logger().info("🔥 fireguard_ros_bridge STARTED (REAL DOCK CONNECTED)")

    # ============================================================
    # CAMERA CALLBACKS
    # ============================================================
    def cb_cam_robot4(self, msg):
        shared_state["camera"]["robot4"] = True
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        shared_state["latest_frame"]["robot4"] = img

    def cb_cam_robot6(self, msg):
        shared_state["camera"]["robot6"] = True
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        shared_state["latest_frame"]["robot6"] = img

    def cb_yolo1(self, msg):
        shared_state["camera"]["cam1"] = True
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        shared_state["latest_frame"]["cam1"] = img

    def cb_yolo2(self, msg):
        shared_state["camera"]["cam2"] = True
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        shared_state["latest_frame"]["cam2"] = img

    # ============================================================
    # DEPTH
    # ============================================================
    def calc_dist(self, depth_img):
        vals = depth_img[depth_img > 0]
        if vals.size == 0:
            return None
        d = float(np.min(vals))
        return d / 1000.0 if d > 50 else d

    def cb_depth_robot4(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        shared_state["depth"]["robot4"] = self.calc_dist(img)

    def cb_depth_robot6(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        shared_state["depth"]["robot6"] = self.calc_dist(img)

    # ============================================================
    # BATTERY
    # ============================================================
    def cb_battery_robot4(self, msg):
        now = time.time()
        if self.last_bat_4 == 0 or now - self.last_bat_4 >= self.bat_interval:
            self.last_bat_4 = now
            shared_state["battery"]["robot4"] = msg.percentage * 100.0

    def cb_battery_robot6(self, msg):
        now = time.time()
        if self.last_bat_6 == 0 or now - self.last_bat_6 >= self.bat_interval:
            self.last_bat_6 = now
            shared_state["battery"]["robot6"] = msg.percentage * 100.0

    # ============================================================
    # POSE
    # ============================================================
    def yaw_from_quat(self, q):
        return math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

    def cb_pose4(self, msg):
        p = msg.pose.pose
        shared_state["robot_pose"]["robot4"] = {
            "x": p.position.x,
            "y": p.position.y,
            "yaw": self.yaw_from_quat(p.orientation),
        }
        shared_state["pose_history_robot4"].append((p.position.x, p.position.y))

    def cb_pose6(self, msg):
        p = msg.pose.pose
        shared_state["robot_pose"]["robot6"] = {
            "x": p.position.x,
            "y": p.position.y,
            "yaw": self.yaw_from_quat(p.orientation),
        }
        shared_state["pose_history_robot6"].append((p.position.x, p.position.y))

    # ============================================================
    # AUDIO
    # ============================================================
    def timer_audio(self):
        audio_state = shared_state["audio"]
        if audio_state["target_gain"] != getattr(self, "last_gain", None):
            self.last_gain = audio_state["target_gain"]
            msg = Int32()
            msg.data = int(self.last_gain)
            self.pub_gain.publish(msg)

        if audio_state["should_beep"]:
            self.pub_beep.publish(Empty())
            audio_state["should_beep"] = False

    # ============================================================
    # FIRE EVENT
    # ============================================================
    def ensure_tables(self):
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS cam_detect (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                topic TEXT NOT NULL,
                message TEXT,
                detect_time DATETIME DEFAULT CURRENT_TIMESTAMP
            );
        """)
        self.conn.commit()

    def cb_fire_event(self, msg):
        raw = msg.data.strip()
        self.cursor.execute("""
            INSERT INTO cam_detect (topic, message, detect_time)
            VALUES (?, ?, CURRENT_TIMESTAMP)
        """, ("/cam/fire_detection_status", raw))
        self.conn.commit()

        shared_state["fire_event"]["detected"] = (raw != "No Detected")
        shared_state["fire_event"]["last_fire_time"] = time.time()

    # ============================================================
    # REAL DOCK EVENT (최종)
    # ============================================================
    def cb_dock_status_real(self, msg, robot_id):
        prev = self.last_dock_state[robot_id]
        curr = bool(msg.is_docked)   # DockStatus 실제 필드
        self.last_dock_state[robot_id] = curr

        if prev is None:
            self.get_logger().info(
                f"{robot_id} initial dock state = {curr} (ignored)"
            )
            return

        # 도킹 상태에서 해제될 때 → 출동 시작 기록 (/api/robot/start)
        if prev is True and curr is False:
            try:
                requests.post(
                    "http://127.0.0.1:5000/api/robot/start",
                    json={"robot_id": robot_id},
                    timeout=1
                )
                self.get_logger().info(
                    f"✅ {robot_id} undock -> Flask mission START"
                )
            except Exception as e:
                self.get_logger().error(
                    f"❌ Flask mission start failed: {e}"
                )

        if prev is False and curr is True:
            try:
                requests.post(
                    "http://127.0.0.1:5000/api/robot/dock",
                    json={"robot_id": robot_id},
                    timeout=1
                )
                self.get_logger().info(
                    f"✅ {robot_id} dock -> Flask DB updated"
                )
            except Exception as e:
                self.get_logger().error(
                    f"❌ Flask dock update failed: {e}"
                )


def start_ros_bridge():
    rclpy.init()
    node = FireGuardBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
