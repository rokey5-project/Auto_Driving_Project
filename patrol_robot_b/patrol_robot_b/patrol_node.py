import rclpy
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
from std_msgs.msg import Bool

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')


        self.create_subscription(Bool, '/cam/fire_detection_status', self.fire_callback, 10)
        self.create_subscription(Bool, "/is_guiding", self.guiding_callback, 10)

        self.position_index = 0
        self.fire_state = False
        self.is_guiding = False

        self.navigator = TurtleBot4Navigator()
        self.navigator.waitUntilNav2Active()
        
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.EAST)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.undock()
        
        self.goal_pose = []

        self.goal_pose.append(self.navigator.getPoseStamped([0.01164, 1.536432], TurtleBot4Directions.SOUTH))
        self.goal_pose.append(self.navigator.getPoseStamped([-2.89687, 1.55269], TurtleBot4Directions.EAST))
        self.goal_pose.append(self.navigator.getPoseStamped([-2.942057, -0.05439], TurtleBot4Directions.NORTH))
        self.goal_pose.append(self.navigator.getPoseStamped([-1.641019, -0.19269], TurtleBot4Directions.EAST))
        self.goal_pose.append(self.navigator.getPoseStamped([-0.015362, 0.440271], TurtleBot4Directions.WEST))
        
        self.create_timer(0.2, self.patrol_loop) 
        

    def fire_callback(self, msg):
        self.fire_state = msg.data
        
    def guiding_callback(self, msg):
        
        if self.is_guiding != msg.data:
            self.is_guiding = msg.data
            
            if self.is_guiding:
                self.get_logger().info("🚨 Person found! Patrol stopped.")
                self.navigator.cancelTask()
        
    def patrol_loop(self):
        if not self.navigator.isTaskComplete():
            self.get_logger().info('On patrol')
            return
            
        if not self.navigator.getDockedStatus() and not self.fire_state:
            self.get_logger().info("🔥 No fire. Docking in progress.")
            self.navigator.dock()
            return
        
        if not self.is_guiding:    
            self.get_logger().info(f"Start to Patrol: Target Pose - {self.position_index}")
            goal = self.goal_pose[self.position_index]
            self.navigator.goToPose(goal)
                
            if self.navigator.getResult() != 2:
                if self.position_index + 1 >= len(self.goal_pose):
                    self.position_index = 0
                else:
                    self.position_index = self.position_index + 1
        
        
def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()