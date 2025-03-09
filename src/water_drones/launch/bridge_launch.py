from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridges = [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/wow_craft/joint/thruster_1_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/wow_craft/joint/thruster_2_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/wow_craft/joint/thruster_3_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double',
                '/model/wow_craft/joint/thruster_4_propeller_joint/cmd_thrust@std_msgs/msg/Float64@gz.msgs.Double'
            ],
            name='thruster_bridge',
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/model/wow_craft/pose@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'
            ],
            name='pose_bridge',
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/waves/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'
            ],
            name='dynamic_pose_bridge',
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/waves/model/wow_craft/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
            ],
            name='joint_state_bridge',
            output='screen',
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/waves/model/wow_craft/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
            ],
            name='imu_sensor_bridge',
            output='screen',
        ),
        Node(
            package='water_drones',
            executable='imu_pose_to_odometry',
            name='imu_pose_to_odometry',
            output='screen',
        ),
    ]

    return LaunchDescription(bridges)