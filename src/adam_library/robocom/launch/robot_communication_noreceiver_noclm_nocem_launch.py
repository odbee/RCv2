from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package= 'robocom',
            executable='gripperexecutionnode',

        ),
        Node(
            package= 'robocom',
            executable='managertogripper',

        ),
        Node(
            package= 'robocom',
            executable='rtdestatepublisher',

        ),
        Node(
            package= 'robocom',
            executable='robotexecutionnode',

        ),
    ])