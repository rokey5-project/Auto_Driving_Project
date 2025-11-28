import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from ultralytics import YOLO
import threading
import time
import cv2
import math
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


class YoloCarNavGoal(Node):
    def __init__(self):
        super().__init__('nav_to_car')

        # Internal state
        self.bridge = CvBridge()
        self.K = None
        self.depth_image = None
        self.rgb_image = None
        self.camera_frame = None
        self.logged_rgb_shape = False

        self.goal_handle = None
        self.shutdown_requested = False
        self.logged_intrinsics = False
        self.current_distance = None
        self.close_enough_distance = 2.0  # meters
        self.block_goal_updates = False
        self.close_distance_hit_count = 0  # To avoid reacting to a single bad reading

        # Load YOLOv8 model
        self.model = YOLO("./yolov8n.pt")

        # ROS 2 TF and Nav2 setup
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Display
        self.display_frame = None
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

        self.navigator = TurtleBot4Navigator()

        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        # ROS 2 subscriptions
        self.create_subscription(CameraInfo, '/robot6/oakd/rgb/camera_info', self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, '/robot6/oakd/rgb/image_raw/compressed', self.rgb_callback, 10)
        self.create_subscription(Image, '/robot6/oakd/stereo/image_raw', self.depth_callback, 10)

        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

        self.last_feedback_log_time = 0

    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")
        self.timer = self.create_timer(0.2, self.process_frame)
        self.start_timer.cancel()

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info(f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, "
                                   f"cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.logged_intrinsics = True

    def rgb_callback(self, msg):
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
            # with self.lock:

            # self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.camera_frame = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"RGB conversion failed: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def process_frame(self):
        if self.K is None or self.rgb_image is None or self.depth_image is None:
            print('program not work' , 'K :', self.K is None, 'rgb :', self.rgb_image is None, 'depth :', self.depth_image is None)
            return

        results = self.model(self.rgb_image, verbose=False)[0]
        frame = self.rgb_image.copy()
        frame_id = getattr(self, 'camera_frame', None)

        t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        robot_x = t.transform.translation.x
        robot_y = t.transform.translation.y
        
        self.display_frame = frame
        target_distance = 0.4

        for det in results.boxes:
            cls = int(det.cls[0])
            label = self.model.names[cls]
            conf = float(det.conf[0])
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

            # Draw box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            if label.lower() == "car":
                u = int((x1 + x2) // 2)
                v = int((y1 + y2) // 2)
                z = float(self.depth_image[v, u])

                fx, fy = self.K[0, 0], self.K[1, 1]
                cx, cy = self.K[0, 2], self.K[1, 2]
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                # z = float(self.depth_image[y, x]) / 1000.0

                print(x, y, z)

                pt_camera = PointStamped()
                pt_camera.header.stamp = rclpy.time.Time().to_msg()
                pt_camera.header.frame_id = frame_id
                pt_camera.point.x = x / 1000
                pt_camera.point.y = y / 1000
                pt_camera.point.z = z / 1000


                pt_map = self.tf_buffer.transform(pt_camera, 'map', timeout=Duration(seconds=1.0))
                self.get_logger().info(f"Map coordinate: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")

                dx = pt_map.point.x - robot_x  # 로봇 현재 위치 필요
                dy = pt_map.point.y - robot_y
                dist = math.hypot(dx, dy)

                if dist > target_distance:
                    scale = (dist - target_distance) / dist
                    goal_x = robot_x + dx * scale
                    goal_y = robot_y + dy * scale
                else:
                    goal_x = robot_x
                    goal_y = robot_y
                    
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = goal_x
                goal_pose.pose.position.y = goal_y
                goal_pose.pose.position.z = 0.0
                yaw = 0.0
                qz = math.sin(yaw / 2.0)
                qw = math.cos(yaw / 2.0)
                robot_z = t.transform.rotation.z
                robot_w = t.transform.rotation.w
                goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=robot_z, w=robot_w)

                self.navigator.goToPose(goal_pose)
                self.get_logger().info("Sent navigation goal to map coordinate.")

    def display_loop(self):
        while rclpy.ok():
            if self.display_frame is not None:
                cv2.imshow("YOLO Detection", self.display_frame)
                key = cv2.waitKey(1)
                if key == 27:  # ESC
                    self.shutdown_requested = True
                    break
                elif key == ord('r'):
                    self.close_distance_hit_count = 0
                    self.get_logger().info("Manual reset: goal updates re-enabled.")
            time.sleep(0.01)



def nav_to_point1():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    # Start on dock
    if not navigator.getDockedStatus():
        navigator.info('Docking before intialising pose')
        navigator.dock()

    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set goal poses
    goal_pose = navigator.getPoseStamped([-2.9, 0.012], TurtleBot4Directions.SOUTH)

    # Undock
    navigator.undock()

    # Go to each goal pose
    navigator.startToPose(goal_pose)

    rclpy.shutdown()


def main():
    # nav_to_point1()
    rclpy.init()
    node = YoloCarNavGoal()

    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
