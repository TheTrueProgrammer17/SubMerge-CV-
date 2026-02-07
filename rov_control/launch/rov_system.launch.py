import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Joystick Node (Input)
    joy_node = Node(
        package='rov_joystick',   # Must match your folder name
        executable='joy_node',    # Must match entry_point in setup.py
        name='joy_node',
        output='screen'
    )

    # 2. Thruster Controller (Logic)
    control_node = Node(
        package='rov_control',    # Must match your folder name
        executable='thruster_controller',
        name='thruster_controller',
        output='screen'
    )

    # 3. Serial Bridge (Talks to ESP32)
    # We will create this next
    serial_node = Node(
        package='rov_control',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        control_node,
        serial_node
    ])
