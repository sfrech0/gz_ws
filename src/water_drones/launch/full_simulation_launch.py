import os
import launch
import launch_ros.actions
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Path to world file
    package_share = get_package_share_directory('water_drones')
    world_file = os.path.join(package_share, 'worlds', '2d_world.sdf')

    # Start Gazebo with world file
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", world_file],
        output="screen"
    )

    # Start the rest with a delay of 10 seconds
    delayed_actions = TimerAction(
        period=10.0,
        actions= [
            LogInfo(msg="Bridge 1 starts now..."),
            ExecuteProcess(
                cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge", 
                     "/drone_1/pose@geometry_msgs/msg/Pose@gz.msgs.Pose"],
                output="screen"
            ),

            LogInfo(msg="Bridge 2 starts now..."),
            ExecuteProcess(
                cmd=["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                     "/drone_2/pose@geometry_msgs/msg/Pose@gz.msgs.Pose"],
                output="screen"
            ),

            LogInfo(msg="Publisher starts now..."),
            ExecuteProcess(
                cmd=["ros2", "run", "water_drones", "csv_to_ros"],
                output= "screen"
            )
        ]
    )

    return launch.LaunchDescription([
        gazebo,
        delayed_actions
    ])



