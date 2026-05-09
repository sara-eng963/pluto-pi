from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get the path to the micro_ros_agent
    uros_ws = os.path.expanduser('~/uros_ws/install/setup.bash')
    
    return LaunchDescription([
        # MicroROS Agent with environment sourced
        ExecuteProcess(
            cmd=['bash', '-c', 
                 f'source {uros_ws} && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200'],
            output='screen'
        ),
        
        # Your communication node
        Node(
            package='pluto',
            executable='comm_test.py',
            output='screen'
        )
    ])
