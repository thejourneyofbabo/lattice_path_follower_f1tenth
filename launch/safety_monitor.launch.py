import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('path_follower_pkg')
    
    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'safety_monitor_config.yaml')
    
    # Safety monitor node
    safety_monitor_node = Node(
        package='path_follower_pkg',
        executable='safety_monitor_node',
        name='safety_monitor',
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        safety_monitor_node
    ])