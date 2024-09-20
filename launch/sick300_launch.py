from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='s300_ros2',
            executable='sick300_driver',
            name='sick300_driver_node',
            parameters=[{
                'devicename': '/dev/ttyUSB0',  # Set device name
                'baudrate': 115200  # Set baud rate
            }]
        )
    ])
