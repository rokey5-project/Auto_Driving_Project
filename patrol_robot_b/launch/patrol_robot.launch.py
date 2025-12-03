from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription([
        # 사람 탐지 노드 (YOLO)
        Node(
            package='patrol_robot_b',
            executable='detect_person_node',
            name='detect_person_node',
            namespace=namespace,
            output='screen',
            remappings=[
                ('/tf', [namespace, '/tf']),
                ('/tf_static', [namespace, '/tf_static'])
            ],
        ),
        
        # 순찰 네비게이션 노드
        Node(
            package='patrol_robot_b',
            executable='patrol_node',
            name='patrol_node',
            namespace=namespace,
            output='screen'
        ),

        # 부저 컨트롤러 노드
        Node(
            package='patrol_robot_b',
            executable='buzzer_node',
            name='buzzer_node',
            namespace=namespace,
            output='screen'
        ),
    ])
