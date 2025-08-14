from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('path_follower_pkg')
    
    # Path to config file
    config_file = PathJoinSubstitution([
        pkg_share, 
        'config', 
        'path_follower_config.yaml'
    ])
    
    # Path follower node
    path_follower_node = Node(
        package='path_follower_pkg',
        executable='path_follower_node',
        name='path_follower',
        parameters=[config_file],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        path_follower_node
    ])