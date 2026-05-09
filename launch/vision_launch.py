from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # USB Camera Node
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            parameters=['/home/pluto/design_ws/src/pluto/config/cam_params.yaml'],
            output='screen',
        ),
        # Vision Node
        Node(
            package='pluto',
            executable='vision_node.py',
            name='vision_node',
            parameters=[{
                'model_path': '/home/pluto/design_ws/src/pluto/models/best.pt',
                'conf_threshold': 0.5
            }],
            output='screen',
        ),

        # Activate vision after 3 seconds
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 3 && ros2 topic pub --once /activate_vision std_msgs/msg/Bool "{data: true}"'],
            output='screen',
        ),

        # Request fruit after 4 seconds
        ExecuteProcess(
            cmd=['bash', '-c', 'sleep 4 && ros2 topic pub --once /from_user std_msgs/msg/String "{data: \"Orange\"}"'],
            output='screen',
        ),
    ])
