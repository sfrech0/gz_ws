#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import subprocess

class DroneToGazebo(Node):
    def __init__(self, models):
        super().__init__('drone_to_gazebo')

        self.models = models

        # Subscribe to the drone's pose topic (PoseStamped message).
        self.subscription = {
            model: self.create_subscription(
                PoseStamped,
                f'/{model}/pose',
                lambda msg, model=model: self.pose_callback(msg, model),
                10
            )
            for model in models
            # self.get_logger().info(f"Subscribed to /{model}/pose.")
        }
            
            

    def pose_callback(self, msg, model):
        self.get_logger().info("Received new PoseStamped message. Forwarding to Gazebo...")
        # Extract the pose from the PoseStamped message.
        pose = msg.pose

        # Build the YAML string required by the Gazebo service.
        request_yaml = (
            f"name: '{model}'," + "  position: {" + f"x: {pose.position.x}, y: {pose.position.y}" + "}"
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

    models = [
        "drone_1", "drone_2"
    ]
    node = DroneToGazebo(models)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
