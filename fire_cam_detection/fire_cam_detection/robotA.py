import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_point
from tf2_ros import TransformException
from ultralytics import YOLO
import threading
import time
import cv2
import math

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration as Dur
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class FireMissionNode(Node):
    def __init__(self):
        super().__init__('fire_mission_node')

        # ===== 내부 상태 =====
        self.bridge = CvBridge()
        self.K = None
        self.depth_image = None
        self.rgb_image = None
        self.camera_frame = None
        self.logged_rgb_shape = False
        self.logged_intrinsics = False

        self.shutdown_requested = False


        # YOLO 모델
        self.model = YOLO(r"/home/rokey/rokey_ws/src/fire_cam_detection/fire_cam_detection/best.pt")
        # self.get_logger().info(f"YOLO model loaded. classes: {self.model.names}")

        # TF 버퍼 & 리스너
        # cache_time 인자 버전마다 달라서 기본 생성자로 두는게 안전
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # TurtleBot4 Navigator
        self.navigator = TurtleBot4Navigator()

        # 시작 시 dock 정리
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        # 초기 포즈 설정
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        # CCTV 타겟 좌표
        self.cam1_target = [-1.98, 2.37]   # A point
        self.cam2_target = [-2.2, 4.63]    # B point

        # fire까지 남길 거리
        self.target_distance = 0.2 # meters

        # 한 번의 사이클만 처리할지 여부
        self.fire_handling_active = False  # fire 처리 중인지
        self.approach_sent = False         # 0.4m 접근 goal 보냈는지

        # qos = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1
        # )

        # OAK-D 카메라 구독
        self.create_subscription(CameraInfo, '/robot4/oakd/rgb/camera_info',
                                 self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, '/robot4/oakd/rgb/image_raw/compressed',
                                 self.rgb_callback, 10)
        self.create_subscription(Image, '/robot4/oakd/stereo/image_raw',
                                 self.depth_callback, 10)

        # CCTV 상태 구독
        self.create_subscription(String,
                                 "/cam/fire_detection_status",
                                 self.fire_status_callback,
                                 10)
        
        self.pub_audio = self.create_publisher(
            AudioNoteVector, 
            '/robot4/cmd_audio', 
            10
        )

        self.get_logger().info("Beep subscriber ready!")

        self.pub_img = self.create_publisher(CompressedImage, 'robotA/yolo/compressed', 10)

        # TF 안정화 후 YOLO→TF→goToPose 시작용 타이머
        self.start_timer = None
        self.process_timer = None

        self.nav_check_timer = None

        # 화면 표시용
        self.display_frame = None
        # self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        # self.display_thread.start()

        self.get_logger().info("FireMissionNode started. Waiting for /cam/fire_detection_status ...")

    # ======================
    # 콜백: 카메라
    # ======================
    def camera_info_callback(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info(
                f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, "
                f"cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}"
            )
            self.logged_intrinsics = True

    def rgb_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            self.rgb_image = rgb

            if rgb is None or rgb.size == 0:
                self.get_logger().error("Decoded RGB image is empty")
            else:
                if not self.logged_rgb_shape:
                    self.get_logger().info(f"RGB image decoded: {rgb.shape}")
                    self.logged_rgb_shape = True

            self.camera_frame = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")

    def depth_callback(self, msg: Image):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    # ======================
    # CCTV 상태 콜백
    # ======================
    def fire_status_callback(self, msg: String):
        text = msg.data.strip()
        # self.get_logger().info(f"[CCTV] Webcam Fire status: {text}")

        if text == "No Detected":
            return

        # 이미 한 번 fire 처리 중이면 무시
        # if self.fire_handling_active:
        #     self.get_logger().info("[CCTV] Already handling a fire event. Ignoring new status.")
        #     return

        # fire 처음 감지됨 → 타겟으로 이동 시작
        if text == "Cam1 Detected":
            target_xy = self.cam1_target
            direction = TurtleBot4Directions.EAST
        elif text == "Cam2 Detected":
            target_xy = self.cam2_target
            direction = TurtleBot4Directions.NORTH
        else:
            self.get_logger().warn(f"[CCTV] Unknown fire status: {text}")
            return

        self.fire_handling_active = True

        # self.navigator.undock()
        self.go_to_target(target_xy, direction)

        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    # ======================
    # 타겟 이동
    # ======================
    def go_to_target(self, target_xy, direction):
        x, y = target_xy
        # Nav2 활성 (amcl 사용)
        self.navigator.waitUntilNav2Active(localizer='amcl')
        goal_pose = self.navigator.getPoseStamped([x, y], direction)
        # 비동기 이동
        self.navigator.startToPose(goal_pose)
        self.get_logger().info(f"Moving to target ({x:.2f}, {y:.2f}) with direction {direction}.")
        

    # ======================
    # TF 안정화 후 시작
    # ======================
    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. YOLO+TF 변환 시작합니다.")
        # 0.2초마다 YOLO+TF 처리 → 0.4m 지점 한 번만 생성하고 타이머 종료
        self.process_timer = self.create_timer(0.2, self.process_frame)
        # start_timer는 한 번만 쓰면 되므로 종료
        if self.start_timer is not None:
            self.start_timer.cancel()
            self.start_timer = None

    def detect_callback(self):
        audio_msg = AudioNoteVector()
        audio_msg.append = False

        audio_msg.notes = [
            AudioNote(
                frequency=880,
                max_runtime=Dur(sec=2, nanosec=0)
            )
        ]

        self.pub_audio.publish(audio_msg)
    # ======================
    # YOLO + TF + 0.4m 목표 생성
    # ======================
    def process_frame(self):
        if self.K is None or self.rgb_image is None or self.depth_image is None:
            print('program not work',
                  'K :', self.K is None,
                  'rgb :', self.rgb_image is None,
                  'depth :', self.depth_image is None)
            return

        # YOLO 추론
        results = self.model(self.rgb_image, classes=0, imgsz=320, conf=0.5, iou=0.5, verbose=False)[0]
        frame = self.rgb_image.copy()
        frame_id = getattr(self, 'camera_frame', None)

        # 로봇 현재 위치 (map 기준)
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                # Time()  # 최신 transform
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f"Transform map->base_link failed: {ex}")
            return

        robot_x = t.transform.translation.x
        robot_y = t.transform.translation.y

        self.display_frame = frame
        target_distance = self.target_distance

        for det in results.boxes:
            cls = int(det.cls[0])
            label = self.model.names[cls]
            conf = float(det.conf[0])
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

            # 박스 그리기
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if label == "0":
                self.detect_callback()
                u = int((x1 + x2) // 2)
                v = int((y1 + y2) // 2)

                # depth 유효성 체크
                z_raw = float(self.depth_image[v, u])
                if z_raw <= 0.0 or math.isnan(z_raw):
                    self.get_logger().warn("Invalid depth at fire center, skip this frame")
                    continue

                z = z_raw

                fx, fy = self.K[0, 0], self.K[1, 1]
                cx, cy = self.K[0, 2], self.K[1, 2]
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                print(x, y, z)

                pt_camera = PointStamped()
                pt_camera.header.stamp = rclpy.time.Time().to_msg()
                pt_camera.header.frame_id = frame_id
                pt_camera.point.x = x / 1000.0
                pt_camera.point.y = y / 1000.0
                pt_camera.point.z = z / 1000.0

                try:
                    pt_map = self.tf_buffer.transform(
                        pt_camera,
                        'map',
                        timeout=Duration(seconds=1.0)
                    )
                except TransformException as ex:
                    self.get_logger().warn(f"Transform camera->map failed: {ex}")
                    return

                self.get_logger().info(
                    f"Map coordinate: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})"
                )

                dx = pt_map.point.x - robot_x
                dy = pt_map.point.y - robot_y
                dist = math.hypot(dx, dy)

                if dist > target_distance:
                    scale = (dist - target_distance) / dist
                    goal_x = robot_x + dx * scale
                    goal_y = robot_y + dy * scale
                else:
                    goal_x = robot_x
                    goal_y = robot_y

                # 0.4m 목표 pose 생성
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = goal_x
                goal_pose.pose.position.y = goal_y
                goal_pose.pose.position.z = 0.0

                yaw = 0.0
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)

                # 로봇 현재 yaw 그대로 유지
                robot_z = t.transform.rotation.z
                robot_w = t.transform.rotation.w
                goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=robot_z, w=robot_w)

                # 한 번만 goToPose 보내고 타이머 종료
                if not self.approach_sent:
                    self.navigator.goToPose(goal_pose)
                    self.get_logger().info("Sent navigation goal to 0.4m from fire (map coordinate).")
                    self.approach_sent = True
                    self.create_timer(30.0, self.finish_work)

                    if self.process_timer is not None:
                        self.process_timer.cancel()
                        self.process_timer = None

                break  # 첫 번째 fire만 처리
        msg1 = self.bridge.cv2_to_compressed_imgmsg(self.display_frame, 'jpg')
        self.pub_img.publish(msg1)

    def finish_work(self):
        self.navigator.goToPose([0,0], TurtleBot4Directions.NORTH)
        self.navigator.dock()


    # ======================
    # 디스플레이 루프
    # ======================
    def display_loop(self):
        while rclpy.ok():
            if self.display_frame is not None:
                cv2.imshow("YOLO Detection", self.display_frame)
                key = cv2.waitKey(1)
                if key == 27:  # ESC
                    self.shutdown_requested = True
                    break
            time.sleep(0.01)

    # ======================
    # 종료 처리
    # ======================
    def destroy_node(self):
        self.get_logger().info("Shutting down FireMissionNode...")
        if self.process_timer is not None:
            self.process_timer.cancel()
        if self.start_timer is not None:
            self.start_timer.cancel()
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = FireMissionNode()

    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
