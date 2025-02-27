#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess

class DroneToGazebo(Node):
    def __init__(self):
        super().__init__('drone_to_gazebo')
        # Subscribe to the drone's pose topic (PoseStamped message).
        self.subscription = self.create_subscription(
            PoseStamped,
            '/drone_1/pose',
            self.pose_callback,
            10
        )
        self.get_logger().info("Subscribed to /drone_1/pose.")

    def pose_callback(self, msg: PoseStamped):
        self.get_logger().info("Received new PoseStamped message. Forwarding to Gazebo...")
        # Extract the pose from the PoseStamped message.
        pose = msg.pose

        # Build the YAML string required by the Gazebo service.
        request_yaml = (
            "name: 'drone_1', position: {" + f"x: {pose.position.x}, y: {pose.position.y}" + "}"
            # "position: "
            # f"x: {pose.position.x},"
            # f"y: {pose.position.y}\n"
            # f"    z: {pose.position.z}\n"
            # "  orientation:\n"
            # f"    x: {pose.orientation.x}\n"
            # f"    y: {pose.orientation.y}\n"
            # f"    z: {pose.orientation.z}\n"
            # f"    w: {pose.orientation.w}"
        )

        self.get_logger().info(f"Sending to Gazebo:\n{request_yaml}")

        # Call the Gazebo service using the gz command.
        cmd = ['gz', 'service', '-s', '/world/waves/set_pose', '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean', '--timeout', '100', '--req',  request_yaml]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            self.get_logger().info(f"Gazebo service call succeeded:\n{result.stdout}")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Gazebo service call failed:\n{e.stderr}")

def main(args=None):
    rclpy.init(args=args)
    node = DroneToGazebo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
