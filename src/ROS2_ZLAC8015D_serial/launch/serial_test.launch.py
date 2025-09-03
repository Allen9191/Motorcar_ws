# ROS2_ZLAC8015D_serial/launch/serial_test.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    keyboard_pkg_share = get_package_share_directory('keyboard_joy')

    keyboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(keyboard_pkg_share, 'launch', 'keyboard_joy.launch.py'))
    )

    simple = Node(
        package='zlac8015d_serial',
        executable='SimpleControl',
        name='simple_control',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([keyboard, simple])
