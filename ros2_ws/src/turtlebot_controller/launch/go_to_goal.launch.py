import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    turtebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        )
    )

    controller_node = Node(
        package='turtlebot_controller',
        executable='go_to_goal',
        name='go_to_goal',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        controller_node
    ])
