import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from std_msgs.msg import Bool

class BuzzerNode(Node):
    def __init__(self):
        super().__init__('buzzer_node')

        # publisher
        self.alarm_publisher = self.create_publisher(AudioNoteVector, 'cmd_audio', 10)
        
        # subscription
        self.create_subscription(Bool, "/is_arrived", self.is_arrived_callback, 10)
        
        self.is_arrived = False

        self.audio_msg = AudioNoteVector()
        self.audio_msg.append = False
        notes = [
            (880, 300000000),
            (440, 300000000),
            (880, 300000000),
            (440, 300000000)
        ]
        for freq, duration_sec in notes:
            note = AudioNote()
            note.frequency = freq
            note.max_runtime.sec = 0
            note.max_runtime.nanosec = duration_sec
            self.audio_msg.notes.append(note)
            
    def is_arrived_callback(self, msg):
        self.is_arrived = msg.data
        
        if self.is_arrived:
            self.alarm_publisher.publish(self.audio_msg)
            self.get_logger().info("🔊 Buzzer Play!")
        
        

def main(args=None):
    rclpy.init(args=args)
    node = BuzzerNode()

    try:
        rclpy.spin(node)  # while 루프 없이 spin()만으로 동작
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
