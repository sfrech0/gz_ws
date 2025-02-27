import launch
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():

    publisher = [
        Node(
            package='water_drones',
            executable='csv_publisher',
            name='csv_publisher',
            output='screen'
        )
    ]

    pose_to_gz = [
        Node(
            package='water_drones',
            executable='pose_to_gz',
            name='pose_to_gz',
            output='screen'
        )
    ]

    # Combine all Nodes
    return LaunchDescription(publisher + pose_to_gz)
