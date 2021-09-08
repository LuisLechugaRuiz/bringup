import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  # Get the task_monitor directory
    params_dir = os.path.join(get_package_share_directory('bringup'), 'params')
    task_monitor_params_file = os.path.join(params_dir, 'task_params.yaml')

    return LaunchDescription([
        Node(
            package='task_monitor',
            executable='task_monitor',
            name='task_monitor',
            output='screen',
            parameters=[task_monitor_params_file])
    ])