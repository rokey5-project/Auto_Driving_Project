import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from std_msgs.msg import Bool
import time


class PatrolNode(Node):
  def __init__(self):
    super().__init('patorl_zone')


    self.create_subscription(Bool, "/person_detected", self.person_callback, 10)
    self.create_subscription(Bool, '/fire_state', self.fire_callback, 10)

    self.position_index = 0
    self.fire_state = False
    self.is_detect_person = False

    self.navigator = TurtleBot4Navigator()
    initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    self.navigator.setInitialPose(initial_pose)
    self.navigator.waitUntilNav2Active()
    self.navigator.undock()

    self.goal_pose = []

    self.goal_pose.append(self.navigator.getPoseStamped([-1.55069, 0.0668084], -2.5896))
    self.goal_pose.append(self.navigator.getPoseStamped([-0.61671, -0.852567], -1.02024))
    self.goal_pose.append(self.navigator.getPoseStamped([0.0343325, -1.96793], 0.21976))
    self.goal_pose.append(self.navigator.getPoseStamped([-0.711899, -0.0612125], 0.606873))

    self.timer = self.create_timer(0.5, self.patrol_loop)

  def person_callback(self, msg):
      self.is_detect_person = msg.data

  def fire_callback(self, msg):
      self.fire_state = msg.data

  def patrol_loop(self):
      if not self.navigator.getDockedStatus() and not self.fire_state:
          self.navigator.dock()
          self.get_logger().info("🔥 화재 없음. 도킹 상태")
          return

      if self.is_detect_person:
          self.get_logger().info("🚨 사람 발견!")
          self.navigator.stop()
          time.sleep(3)

      # 순찰 진행
      goal = self.goals[self.index]
      self.navigator.startToPose(goal)
      self.get_logger().info(f"순찰 진행: 목표 지점 {self.index}")
      self.index = (self.index + 1) % len(self.goals)

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