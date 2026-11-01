from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # RPLidar A1
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),

        # IMU BNO055
        Node(
            package='imu_bno055',
            executable='bno055_node',
            name='bno055',
            output='screen',
        ),

        # MCL Navigator
        Node(
            package='mcl_nav',
            executable='navigator',
            name='navigator',
            output='screen',
        ),
    ])
