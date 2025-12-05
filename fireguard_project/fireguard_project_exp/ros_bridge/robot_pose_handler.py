# ros_bridge/robot_pose_handler.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time
from ros_bridge.utils import shared_state



class RobotPoseHandler(Node):
    def __init__(self):
        super().__init__("robot_pose_handler")

        self.last_r4 = 0
        self.last_r6 = 0

        # 로봇4
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/robot4/amcl_pose",
            self.cb_r4,
            10
        )

        # 로봇6
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/robot6/amcl_pose",
            self.cb_r6,
            10
        )

        self.get_logger().info("RobotPoseHandler STARTED (AMCL mode)")

    # --- quaternion → yaw ---
    def quat_to_yaw(self, q):
        return math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2 * (q.y*q.y + q.z*q.z)
        )

    # --- extract pose ---
    def extract(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = self.quat_to_yaw(msg.pose.pose.orientation)
        return x, y, yaw

    # --- robot4 callback ---
    def cb_r4(self, msg):
        now = time.time()
        if now - self.last_r4 < 0.05:  # 20Hz
            return
        self.last_r4 = now

        x, y, yaw = self.extract(msg)
        shared_state["robot_pose"]["robot4"] = {"x": x, "y": y, "yaw": yaw}

    # --- robot6 callback ---
    def cb_r6(self, msg):
        now = time.time()
        if now - self.last_r6 < 0.05:
            return
        self.last_r6 = now

        x, y, yaw = self.extract(msg)
        shared_state["robot_pose"]["robot6"] = {"x": x, "y": y, "yaw": yaw}


def main():
    rclpy.init()
    node = RobotPoseHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
