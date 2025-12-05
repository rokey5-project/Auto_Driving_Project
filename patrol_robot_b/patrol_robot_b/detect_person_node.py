import threading
import time
import cv2
import math
import numpy as np
from ultralytics import YOLO
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
import tf2_ros
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from std_msgs.msg import Bool
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
import tf2_geometry_msgs 


class DetectPersonNode(Node):
    def __init__(self):
        super().__init__('detect_person_node')

        self.bridge = CvBridge()
        self.K = None
        self.depth_image = None
        self.rgb_image = None
        self.camera_frame = None
        self.rgb_image_stamp = None
        self.shutdown_requested = False
        self.goal_pose = None
        
        self.is_navigating = False # 이동 중 여부
        self.is_arrived = False # 목적이 도착 여부
        self.is_guiding = False # 대피소 안내 여부 (True: 대피소 이동중, False: 순찰중)

        self.navigator = TurtleBot4Navigator()

        self.model = YOLO("./yolov8n.pt")
        
        # buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Display
        self.display_frame = None
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()

        # publisher
        self.arrived_pub = self.create_publisher(Bool, "/is_arrived", 10)
        self.guiding_pub = self.create_publisher(Bool, "/is_guiding", 10)
        
        # subscriptions
        self.create_subscription(CameraInfo, '/robot6/oakd/rgb/camera_info', self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, '/robot6/oakd/rgb/image_raw/compressed', self.rgb_callback, 10)
        self.create_subscription(Image, '/robot6/oakd/stereo/image_raw', self.depth_callback,10)

        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        
        # timer
        self.start_timer = self.create_timer(5.0, self.start_transform)
        self.guiding_timer = self.create_timer(0.2, self.publishing_guiding)
        self.arrived_timer = self.create_timer(0.2, self.publishing_arrived) 

    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")
        self.timer = self.create_timer(0.3, self.process_frame)
        self.start_timer.cancel()

    def camera_info_callback(self, msg):
        try:
            self.K = np.array(msg.k).reshape(3, 3)
        except Exception as e:
            self.get_logger().error(f"카메라 정보 데이터 변환에 실패하였습니다.: {e}")

    def rgb_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            self.rgb_image = rgb
            self.camera_frame = msg.header.frame_id
            self.rgb_image_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().error(f"RGB 이미지 변환에 실패하였습니다.: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth 이미지 변환에 실패하였습니다.:{e}")

    def process_frame(self):
        if self.K is None or self.rgb_image is None or self.depth_image is None:
            self.get_logger().error(f"k : {self.K is None}, rgb: {self.rgb_image is None}, depth: {self.depth_image is None}")
            return
        
        frame = self.rgb_image.copy()
        frame_id = getattr(self, 'camera_frame', None)
        h, w, _ = frame.shape
        
        self.display_frame = frame
        
        # 구역 외 사람 detect 방지하기 위한 이미지 cropping
        y_start = int(0.4 * h) 
        cropped_frame = frame[y_start:h, 0:w]
        
        # predict
        results = self.model(cropped_frame, conf=0.7, verbose=False)[0]

        try:
            t = self.tf_buffer.lookup_transform(
                'map', 
                'base_link',
                rclpy.time.Time()
            )
        except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f"TF 룩업 시간 문제: {e}")
            return
        
        # robot position
        robot_x = t.transform.translation.x
        robot_y = t.transform.translation.y

        # 타겟 위치와 일정 거리
        self.target_distance = 0.6
        
        for det in results.boxes:
            for i in range(len(det.cls)):
                cls = int(det.cls[i])
                label = self.model.names[cls]

                # 사람 감지
                if label.lower() == "person":
                    conf = float(det.conf[0])
                    x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

                    original_y1 = y1 + y_start
                    original_y2 = y2 + y_start

                    cv2.rectangle(frame, (x1, original_y1), (x2, original_y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} {conf:.2f}", (x1, original_y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                    # object center pose
                    u = int((x1 + x2) // 2)
                    v = int((original_y1 + original_y2) // 2)
                    z = float(self.depth_image[v, u])
                    
                    # intrinsic 변환
                    fx, fy = self.K[0, 0], self.K[1, 1]
                    cx, cy = self.K[0, 2], self.K[1, 2]
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy

                    pt_camera = PointStamped()
                    pt_camera.header.stamp = rclpy.time.Time().to_msg()
                    pt_camera.header.frame_id = frame_id
                    pt_camera.point.x = x / 1000
                    pt_camera.point.y = y / 1000
                    pt_camera.point.z = z / 1000


                    pt_map = self.tf_buffer.transform(pt_camera, 'map', timeout=Duration(seconds=1.0))
                    self.get_logger().info(f"Map coordinate: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")

                    dx = pt_map.point.x - robot_x
                    dy = pt_map.point.y - robot_y
                    dist = math.hypot(dx, dy)
                    
                    if dist > self.target_distance:
                        scale = (dist - self.target_distance) / dist
                        goal_x = robot_x + dx * scale
                        goal_y = robot_y + dy * scale
                    else:
                        goal_x = robot_x
                        goal_y = robot_y
                        
                    self.goal_pose = PoseStamped()
                    self.goal_pose.header.frame_id = 'map'
                    self.goal_pose.header.stamp = self.get_clock().now().to_msg()
                    self.goal_pose.pose.position.x = goal_x
                    self.goal_pose.pose.position.y = goal_y
                    self.goal_pose.pose.position.z = 0.0

                    robot_z = t.transform.rotation.z
                    robot_w = t.transform.rotation.w
                    self.goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=robot_z, w=robot_w)
                    
                    self.display_frame = frame
                    
                    # 순찰중 사람 발견
                    if not self.is_guiding:
                        self.get_logger().info(f'person detected')
                        self.is_guiding = True
                        
                        # patrol navigation cancel 되기까지 delay
                        self.target_timer = self.create_timer(1, self.goToTarget)
                        self.target_timer.cancel()

                if self.is_guiding:
                    # UNKNOWN = 0
                    # SUCCEEDED = 1
                    # CANCELED = 2
                    # FAILED = 3
                    if self.navigator.getResult() == 1:
                        self.get_logger().info("Arrived completed")
                        self.is_arrived = True
                        
                        self.shelter_timer = self.create_timer(3, self.goToShelter)
                        
                    else:
                        self.is_arrived = False
                        self.get_logger().info(f"Moving to Target / robot_pose : {robot_x}, {robot_y}")

    def goToTarget(self):
        self.self.navigator.goToPose(self.goal_pose)
        
    def goToShelter(self):
        # 대피소 위치
        shelter_pose = self.navigator.getPoseStamped([1.8689, 1.4689], TurtleBot4Directions.SOUTH)
        self.navigator.goToPose(shelter_pose)
        
        self.get_logger().info("Moving to Shelter")
        self.shelter_timer.cancel()
        
        # 좌표 이동 완료
        if self.navigator.getResult() == 1:
            self.is_arrived = True
            self.is_guiding = False
            
    def display_loop(self):
        while rclpy.ok():
            if self.display_frame is not None:
                cv2.imshow("YOLO Detection", self.display_frame)
                key = cv2.waitKey(1)
                if key == 27:  # ESC
                    self.shutdown_requested = True
                    break
            time.sleep(0.01)
            
    def publishing_guiding(self):
        guiding_msg = Bool()
        guiding_msg.data = self.is_guiding
        self.guiding_pub.publish(guiding_msg)
            
    def publishing_arrived(self):
        arrived_msg = Bool()
        arrived_msg.data = self.is_arrived
        self.arrived_pub.publish(arrived_msg)

def main():        
    rclpy.init()
    node = DetectPersonNode()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
