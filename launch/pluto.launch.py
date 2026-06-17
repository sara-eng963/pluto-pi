from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    cleanup_ros_processes = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "echo '[] Killing any leftover ROS processes...' && "
            "pkill -f '[r]osbridge_websocket' || true; "
            "pkill -f '[r]osbridge_server' || true; "
            "pkill -f '[m]icro_ros_agent' || true; "
            "echo '[] Waiting for serial ports to release...' && "
            "sleep 2",
        ],
        output="screen",
    )

    micro_ros_agent_usb0 = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "echo '[] Sourcing ROS...' && "
            "source /opt/ros/jazzy/setup.bash && "
            "source ~/uros_ws/install/local_setup.bash && "
            "echo '[] Starting micro-ROS agent on /dev/ttyUSB0...' && "
            "exec ros2 run micro_ros_agent micro_ros_agent serial "
            "--dev /dev/ttyUSB0 -b 115200",
        ],
        output="screen",
    )

    micro_ros_agent_usb1 = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "echo '[] Starting micro-ROS agent on /dev/ttyUSB1...' && "
            "source /opt/ros/jazzy/setup.bash && "
            "source ~/uros_ws/install/local_setup.bash && "
            "exec ros2 run micro_ros_agent micro_ros_agent serial "
            "--dev /dev/ttyUSB1 -b 115200",
        ],
        output="screen",
    )

    pluto_nodes = [
        Node(
            package="pluto",
            executable="mission_node.py",
            name="mission_node",
            output="screen",
        ),
        Node(
            package="pluto",
            executable="obstacle_node.py",
            name="obstacle_node",
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

    return LaunchDescription(
        [
            cleanup_ros_processes,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=cleanup_ros_processes,
                    on_exit=[
                        micro_ros_agent_usb0,
                        TimerAction(period=1.0, actions=[micro_ros_agent_usb1]),
                        TimerAction(period=3.0, actions=pluto_nodes),
                    ],
                )
            ),
        ]
    )
