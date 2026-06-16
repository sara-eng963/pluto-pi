from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pluto",
                executable="mission_node.py",
                name="mission_node",
                output="screen",
            ),
            Node(
                package="pluto",
                executable="navigation_node.py",
                name="navigation_node",
                output="screen",
            ),
            Node(
                package="pluto",
                executable="rfid_node.py",
                name="rfid_node",
                output="screen",
            ),
           Node(
                 package="pluto",
                 executable="gui_node.py",
                 name="gui_node",
                 output="screen",
             ),
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    "source ~/vision_env/bin/activate && exec ros2 run pluto vision_node.py",
                ],
                output="screen",
            ),
        ]
    )
