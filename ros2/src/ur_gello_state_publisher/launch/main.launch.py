from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name="com_port",
            default_value="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9HD7RD-if00-port0", # TODO: get from
            description="Serial COM port for scaled-down UR GELLO"
        ),
        DeclareLaunchArgument(
            name="publish_rate",
            default_value="250.0",
            description="Publishing rate for the state publisher (Hz)"
        ),
        Node(
            package="ur_gello_state_publisher",
            executable="gello_publisher",
            name="gello_publisher",
            output="screen",
            parameters=[
                {"com_port": LaunchConfiguration("com_port")},
                {"publish_rate": LaunchConfiguration("publish_rate")}
            ]
        )
    ])
