import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
import time
from .utils import shared_state


class BatteryHandler(Node):
    def __init__(self):
        super().__init__("battery_handler")

        # ★ 1분(60초)에 한 번만 업데이트
        self.throttle_sec = 60.0
        self.last_update_A = 0.0
        self.last_update_B = 0.0

        # SUBSCRIBE
        self.create_subscription(
            BatteryState,
            "/robot4/battery_state",
            self.cb_battery_A,
            10
        )

        self.create_subscription(
            BatteryState,
            "/robot6/battery_state",
            self.cb_battery_B,
            10
        )

    def cb_battery_A(self, msg):
        now = time.time()

        # ★ throttle: 60초 지나지 않았으면 무시
        if now - self.last_update_A < self.throttle_sec:
            return

        self.last_update_A = now
        shared_state["battery"]["A"] = msg.percentage * 100.0

        self.get_logger().info(
            f"(1분 주기) Robot4 (A) Battery: {msg.percentage*100:.1f}%"
        )

    def cb_battery_B(self, msg):
        now = time.time()

        # ★ throttle: 60초 지나지 않았으면 무시
        if now - self.last_update_B < self.throttle_sec:
            return

        self.last_update_B = now
        shared_state["battery"]["B"] = msg.percentage * 100.0

        self.get_logger().info(
            f"(1분 주기) Robot6 (B) Battery: {msg.percentage*100:.1f}%"
        )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryHandler()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
