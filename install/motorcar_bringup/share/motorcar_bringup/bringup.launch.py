# motorcar_bringup/launch/all_bringup.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # 轉傳給 joystick_motor.launch.py 的參數
    dev_arg = DeclareLaunchArgument('dev', default_value='/dev/input/js0')
    dz_arg  = DeclareLaunchArgument('deadzone', default_value='0.05')
    ar_arg  = DeclareLaunchArgument('autorepeat_rate', default_value='0.0')

    # 1) nav_sim 的 NAV_cmd 節點
    nav_cmd = Node(
        package='nav_sim',
        executable='NAV_cmd',     # 確認你的目標可執行檔名
        name='nav_cmd',
        output='screen',
        emulate_tty=True
    )

    # 2) 包含 zlac8015d_serial 的 joystick_motor 啟動（你前面已寫好）
    serial_pkg_share = get_package_share_directory('zlac8015d_serial')
    joystick_motor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(serial_pkg_share, 'launch', 'joystick_motor.launch.py')),
        launch_arguments={
            'dev': LaunchConfiguration('dev'),
            'deadzone': LaunchConfiguration('deadzone'),
            'autorepeat_rate': LaunchConfiguration('autorepeat_rate'),
        }.items()
    )

    return LaunchDescription([
        dev_arg, dz_arg, ar_arg,
        nav_cmd,
        joystick_motor
    ])
