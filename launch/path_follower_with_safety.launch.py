import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('path_follower_pkg')
    
    # Path to config files
    path_follower_config = os.path.join(pkg_dir, 'config', 'path_follower_config.yaml')
    safety_monitor_config = os.path.join(pkg_dir, 'config', 'safety_monitor_config.yaml')
    
    # Path follower node
    path_follower_node = Node(
        package='path_follower_pkg',
        executable='path_follower_node',
        name='path_follower',
        parameters=[path_follower_config],
        output='screen',
        emulate_tty=True,
    )
    
    # Safety monitor node  
    safety_monitor_node = Node(
        package='path_follower_pkg',
        executable='safety_monitor_node',
        name='safety_monitor',
        parameters=[safety_monitor_config],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        path_follower_node,
        safety_monitor_node
    ])