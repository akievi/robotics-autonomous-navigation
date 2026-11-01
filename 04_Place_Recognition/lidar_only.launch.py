"""
Startet NUR den RPLidar-Node (keine IMU, kein Navigator) -- fuer Place-Recognition-Tests,
bei denen der Roboter manuell positioniert wird und NICHT autonom losfahren soll.
Gleiche Parameter wie der Lidar-Teil in mcl_nav/launch/mcl_nav.launch.py (nur lesend
uebernommen, jene Datei bleibt unveraendert).

Nutzung:
    ros2 launch /home/pi/ros2_ws/src/mcl_nav/place_recognition_dev/lidar_only.launch.py
"""

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
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
