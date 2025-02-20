#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose

def test_service_call():
    rclpy.init()
    node = Node('test_service_client')

    client = node.create_client(SetEntityState, '/gazebo/set_model_state')
    while not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().info("Waiting for service...")

    request = SetEntityState.Request()
    request.state.name = 'drone_1'  # Make sure this matches Gazebo model name
    request.state.pose.position.x = 2.0
    request.state.pose.position.y = 3.0
    request.state.pose.position.z = 5.0
    request.state.reference_frame = 'world'

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    if future.result():
        node.get_logger().info(f"Response: {future.result().success}")
    else:
        node.get_logger().error("Failed to call service!")

    node.destroy_node()
    rclpy.shutdown()

def main(args=None):
    test_service_call()

if __name__ == '__main__':
    main()
