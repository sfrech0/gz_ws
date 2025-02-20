import os
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess
from launch import LaunchDescription



def generate_launch_description():

    # Get the absolute path to the water_drones package
    package_share_directory = get_package_share_directory("water_drones")

    # Construct the full path to the world file (assuming it's in the 'worlds' folder)
    # world_file_path = os.path.join(package_share_directory, "worlds", "2d_world.sdf")
    world_file_path = "/home/sfrech/ros2_ws/src/water_drones/worlds/2d_world.sdf"

 
    return LaunchDescription([
        ExecuteProcess(
            cmd=["gzserver", world_file_path, "--verbose"],
            output="screen"
        ),
        ExecuteProcess(
            cmd=["gzclient"],
            output="screen"
        )
    ])




# def generate_launch_description():
#     world_file = os.path.join(get_package_share_directory('water_drones'), 'worlds', '2d_world.sdf')

#     return launch.LaunchDescription([
#         DeclareLaunchArgument(
#             'world',
#             default_value=world_file,
#             description='Full path to world file'
#         ),
#         launch_ros.actions.Node(
#             package='gazebo_ros',
#             executable='gzserver',
#             output='screen',
#             arguments=['--verbose', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')]
#         ),
#         launch_ros.actions.Node(
#             package='gazebo_ros',
#             executable='gzclient',
#             output='screen'
#         ),
#     ])
