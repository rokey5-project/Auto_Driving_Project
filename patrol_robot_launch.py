from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 순찰 네비게이션 노드
        Node(
            package='my_robot_pkg',
            executable='patrol_node',
            name='patrol_node',
            output='screen'
        ),

        # 사람 탐지 노드 (YOLO)
        Node(
            package='my_robot_pkg',
            executable='detect_person_node',
            name='detect_person_node',
            output='screen'
        ),

        # 부저 컨트롤러 노드
        Node(
            package='my_robot_pkg',
            executable='buzzer_node',
            name='buzzer_node',
            output='screen'
        ),
    ])
