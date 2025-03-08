from launch import LaunchDescription
from launch_ros.actions import Node



def generate_launch_description():

    # launch the motor controller: velocity to motor commands
    motor = [
        Node(
            package='water_drones',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
        )
    ]

    bridge = [
        Node(
            package='launch',
            executable='bridge_launch',
            name='bridge_launch',
            output='screen',
        )
    ]

    controller = [
        Node(
            package='water_drones',
            executable='timed_pursuit_node',
            name='timed_pursuit_node',
            output='screen',
        )
    ]

    return LaunchDescription(motor + bridge + controller)