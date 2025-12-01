import rclpy
from rclpy.node import Node
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from std_msgs.msg import Bool

class BuzzerNode(Node):
    def __init__(self):
        super().__init__('buzzer_node')

        self.publisher_ = self.create_publisher(AudioNoteVector, '/cmd_audio', 10)
        self.create_subscription(Bool, "/person_detected", self.person_detected_callback, 10)
        self.is_detect_person = False

        self.audio_msg = AudioNoteVector()
        self.audio_msg.append = False

        notes = [
            (880, 0.3),
            (440, 0.3),
            (880, 0.3),
            (440, 0.3)
        ]

        for freq, duration_sec in notes:
            note = AudioNote()
            note.frequency = freq
            note.max_runtime.sec = 0
            note.max_runtime.nanosec = int(duration_sec * 1e9)
            self.audio_msg.notes.append(note)

        # 일정 주기로 반복 재생
        self.timer = self.create_timer(1.2, self.play_siren)

    def person_detected_callback(self, msg):
      self.is_detect_person = msg.data

    def play_siren(self):
        if self.is_detect_person:
          self.publisher_.publish(self.audio_msg)
          self.get_logger().info("🔊 삐뽀 재생!")

def main(args=None):
    rclpy.init(args=args)
    node = BuzzerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
