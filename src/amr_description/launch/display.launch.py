from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    # 允許覆寫 URDF 路徑（預設用套件內的）
    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value=os.path.join(
            get_package_share_directory('amr_description'),
            'urdf', 'structure_link.urdf.xacro'),
        description='Path to xacro file'
    )
    urdf_path = LaunchConfiguration('urdf')

    # 解析 xacro -> xml（robot_description）
    doc = xacro.process_file(os.path.join(
        get_package_share_directory('amr_description'),
        'urdf', 'structure_link.urdf.xacro'))
    robot_desc = {'robot_description': doc.toxml()}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_desc],
    )

    return LaunchDescription([urdf_arg, rsp])
