# ROS2_ZLAC8015D_serial/launch/joystick_motor.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    dev_arg   = DeclareLaunchArgument('dev',   default_value='/dev/input/js0')
    dz_arg    = DeclareLaunchArgument('deadzone', default_value='0.05')
    ar_arg    = DeclareLaunchArgument('autorepeat_rate', default_value='0.0')
    port_f    = DeclareLaunchArgument('port_f', default_value='/dev/ttyUSB0')
    port_r    = DeclareLaunchArgument('port_r', default_value='/dev/ttyUSB1')

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'dev': LaunchConfiguration('dev'),
            'deadzone': LaunchConfiguration('deadzone'),
            'autorepeat_rate': LaunchConfiguration('autorepeat_rate'),
        }],
        #prefix=['xterm -T JOY_node -e'],
        #prefix=['gnome-terminal --title=JOY_node -- bash -lc'],
        
    )

    joy_control = Node(
        package='joy_control',
        executable='joy_control',
        name='joy_control',
        output='screen',
        emulate_tty=True,
        #prefix=['xterm -T JOY_pub -e'],
        #prefix=['gnome-terminal --title=JOY_ctl -- bash -lc'],

    )

    motorctl = Node(
        package='zlac8015d_serial',
        executable='Motorctl',
        name='motorctl',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'port_f': LaunchConfiguration('port_f'),
            'port_r': LaunchConfiguration('port_r'),
        }],
        #prefix=['xterm -T Motorctl -e'],
        #prefix=['gnome-terminal --title=Motorctl -- bash -lc'],
    )
    
    motorrcv = Node(
        package='zlac8015d_serial',
        executable='Motorrcv',
        name='motorrcv',
        output='screen',
        emulate_tty=True,
        #prefix=['xterm -T Motorctl -e'],
        #prefix=['gnome-terminal --title=Motorrcv -- bash -lc'],
    )

    return LaunchDescription([dev_arg, dz_arg, port_f, port_r, ar_arg, joy_node, joy_control, motorctl, motorrcv])
