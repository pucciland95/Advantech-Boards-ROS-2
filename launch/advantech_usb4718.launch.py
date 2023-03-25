from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    board_name = LaunchConfiguration("board_name").perform(context)

    robot = Node(
        package="advantech_usb4718",
        executable="DAQ_Node",
        name="advantech_usb4718",
        output="screen",
        arguments=[board_name]
    )

    return [robot]


def generate_launch_description():

    # Launch arguments
    launch_args = []

    # Board name
    launch_args.append(
        DeclareLaunchArgument(
            name="board_name",
            default_value="USB-4718,BID#1",
            description="Board name. The one we have at Merlin Lab is called: USB-4718,BID#1",
        )
    )

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])