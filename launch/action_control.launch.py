from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 액션 컨트롤 노드 실행
    action_control_node = Node(
        package='action_control',
        executable='action_control_node',
        name='action_control_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )
    
    intersection_check_node = Node(
        package='action_control',
        executable='intersection_check_node',
        name='intersection_check_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )
    
    return LaunchDescription([
        action_control_node,
        intersection_check_node,
    ])
