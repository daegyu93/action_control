from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 액션 서버 노드들 직접 실행
    line_trace_server = Node(
        package='gole_action',
        executable='line_trace_action_server',
        name='line_trace_action_server',
        parameters=[{
            'linear_speed': 0.25,
            'angular_p_gain': 0.6,
            'max_angular_speed': 0.5,
            'use_sim_time': True,
        }],
        remappings=[
            ('/lsd_markers', '/camera/color/lsd_markers'),
        ],
        output='screen',
    )
    
    move_robot_server = Node(
        package='gole_action',
        executable='move_robot_action_server',
        name='move_robot_action_server',
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ('/odom', '/diff_drive_controller/odom'),
        ],
        output='screen',
    )
    
    rotate_robot_server = Node(
        package='gole_action',
        executable='rotate_robot_action_server',
        name='rotate_robot_action_server',
        parameters=[{
            'use_sim_time': True,
        }],
        remappings=[
            ('/odom', '/diff_drive_controller/odom'),
            ('/imu', '/imu_plugin/out'),
        ],
        output='screen',
    )
    
    # 교차로 처리 노드 실행
    intersection_handler_node = Node(
        package='action_control',
        executable='intersection_handler_node',
        name='intersection_handler_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )
    
    return LaunchDescription([
        line_trace_server,
        move_robot_server,
        rotate_robot_server,
        intersection_handler_node,
    ])
