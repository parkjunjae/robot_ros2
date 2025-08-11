from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
    
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'frame_id': 'odom',   # 출력 맵 프레임
                'resolution': 0.03,
                'publish_2d_map': True,
                'publish_2d_map_period_sec': 1.0,
                'occupancy_min_z': 1.0,
                'occupancy_max_z': 1.2,
                'pointcloud_min_z': 0.5,
                'pointcloud_max_z': 1.2,
                'sensor_model.max_range': 10.0,
                'sensor_model.min_range': 1.0,
                'sensor_model.hit_prob': 0.7,
                'sensor_model.miss_prob': 0.4,
                'sensor_model.clamping_thres_min': 0.12,
                'sensor_model.clamping_thres_max': 0.97,
                'queue_size': 100
            }],
            remappings=[
                ('cloud_in', '/Laser_map'),  # 입력 포인트클라우드 토픽
            ]
        ),
        
        # Node(
        #     package="pointcloud_to_laserscan",
        #     executable="pointcloud_to_laserscan_node",
        #     name="pointcloud_to_laserscan",
        #     remappings=[
        #         ("cloud_in", "/Laser_map"),
        #         ("scan", "/scan")
        #     ],
        #     parameters=[{
        #         "target_frame": "base_link",
        #         "transform_tolerance": 0.01,
        #         "min_height": 0.1,
        #         "max_height": 1.5,
        #         "angle_min": -3.14,
        #         "angle_max": 3.14,
        #         "angle_increment": 0.0087,
        #         "scan_time": 0.1,
        #         "range_min": 0.3,
        #         "range_max": 15.0,
        #         "use_inf": True,
        #     }]
        # ),
        
         # amcl localization
        # Node(
        #     package='nav2_amcl',
        #     executable='amcl',
        #     name='amcl',
        #     output='screen',
        #     parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml']
        # ),
        
        
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml']),

        # controller
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml']),
        
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen',
             parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml']),

        # behavior_server (★ /cmd_vel -> /cmd_vel_nav 로 remap)
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml'],
             remappings=[('/cmd_vel', '/cmd_vel_nav')]),

        # velocity_smoother (★ 입력: /cmd_vel_nav, 출력: /cmd_vel)
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen',
             parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml'],
             remappings=[('/cmd_vel', '/cmd_vel_nav'),
                         ('/cmd_vel_smoothed', '/cmd_vel')]),

        # bt_navigator, waypoint_follower, lifecycle_manager
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml']),
        
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=['/home/vertin/ros2_ws/src/your_nav2_pkg/config/nav2_params.yaml']),
        
        Node(package='nav2_lifecycle_manager',
             executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{
                 'autostart': True,
                 'bond_disable_heartbeat_timeout': True,
                 'node_names': [
                     'controller_server','planner_server',
                     'behavior_server','bt_navigator',
                     'waypoint_follower','velocity_smoother',
                     'smoother_server'
                 ]
             }]),
        
    ])
