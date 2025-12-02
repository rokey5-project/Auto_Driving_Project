import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from std_msgs.msg import Bool
import time


class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')


        self.create_subscription(Bool, "/person_detected", self.person_callback, 10)
        self.create_subscription(Bool, '/fire_state', self.fire_callback, 10)

        self.position_index = 0
        # 토픽 받아오는걸로 변경 필요
        self.fire_state = True
        self.is_detect_person = False

        self.navigator = TurtleBot4Navigator()
        self.navigator.waitUntilNav2Active()
        
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.EAST)
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("Nav2 active, waiting for initial pose...")
        self.navigator.undock()

        self.goal_pose = []

        self.goal_pose.append(self.navigator.getPoseStamped([0.01164, 1.536432], TurtleBot4Directions.SOUTH))
        self.goal_pose.append(self.navigator.getPoseStamped([-2.89687, 1.55269], TurtleBot4Directions.EAST))
        self.goal_pose.append(self.navigator.getPoseStamped([-2.942057, -0.05439], TurtleBot4Directions.NORTH))
        self.goal_pose.append(self.navigator.getPoseStamped([-1.641019, -0.19269], TurtleBot4Directions.EAST))
        self.goal_pose.append(self.navigator.getPoseStamped([-0.015362, 0.440271], TurtleBot4Directions.WEST))

        self.timer = self.create_timer(0.5, self.patrol_loop)
        

    def person_callback(self, msg):
        self.is_detect_person = msg.data
        
    def fire_callback(self, msg):
        self.fire_state = msg.data

    def patrol_loop(self):
        # patrol 종료
        if not self.navigator.getDockedStatus() and not self.fire_state:
            self.navigator.dock()
            self.get_logger().info("🔥 화재 없음. 도킹 상태")
            return
        
        # person detect 시 일시 정지
        if self.is_detect_person:
            self.get_logger().info("🚨 사람 발견! 순찰 정지")
            self.navigator.stop()
        
        if self.navigator.isTaskComplete():
            self.position_index = (self.position_index + 1) % len(self.goal_pose)
            goal = self.goal_pose[self.position_index]
            
            self.navigator.startToPose(goal)
            self.get_logger().info(f"순찰 진행: 목표 지점 {self.position_index}")
        
def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()