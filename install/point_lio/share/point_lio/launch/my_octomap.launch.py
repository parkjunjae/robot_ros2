from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'map',   # 출력 맵 프레임
                'min_z': -15000.0,
                'max_z': 15000.0,
            }],
            remappings=[
                ('cloud_in', '/unilidar/cloud'),  # 입력 포인트클라우드 토픽
            ]
        )
    ])

