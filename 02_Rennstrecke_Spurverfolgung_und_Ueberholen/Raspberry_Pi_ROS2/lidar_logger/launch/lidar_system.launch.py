from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),

    # Filter scan to ±10 degrees
    Node(
        package='lidar_logger',
        executable='lidar_filter',
        name='lidar_filter',
        output='screen',
        parameters=[{
            'angle_begin': -20.0,
            'angle_end': 20.0,
        }],
    ),

    # Obstacle detector
    Node(
        package='lidar_logger',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        parameters=[{
            'detection_distance': 0.35,
        }],
    ),
])


