# send_trajectory.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ur_gello_state_publisher",
            executable="gello_to_ur_trajectory",
            name="gello_to_ur_trajectory",
            output="screen"
        )
    ])
