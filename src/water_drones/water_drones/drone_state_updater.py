import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.srv import SetEntityPose

class DroneStateUpdater(Node):
    def __init__(self):
        super().__init__('drone_state_updater')

        # Create a client for the set_entity_pose service
        self.client = self.create_client(SetEntityPose, 'set_entity_pose')

        # Wait for the service to be available
        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for set_entity_pose service...')
        self.get_logger().info('Service available, starting subscriptions...')

        # Subscribe to the pose topics for each drone
        self.create_subscription(PoseStamped, '/drone_1/pose', self.drone1_callback, 10)
        self.create_subscription(PoseStamped, '/drone_2/pose', self.drone2_callback, 10)

    def drone1_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received position for drone_1: {msg.pose.position.x}, {msg.pose.position.y}")
        self.update_model_state('drone_1', msg)

    def drone2_callback(self, msg: PoseStamped):
        self.get_logger().info(f"Received position for drone_2: {msg.pose.position.x}, {msg.pose.position.y}")
        self.update_model_state('drone_2', msg)

    def update_model_state(self, model_name: str, pose_msg: PoseStamped):
        # Create a service request to update the model state
        req = SetEntityPose.Request()
        req.entity.name = model_name  # Model name must match the name in your world
        req.pose = pose_msg.pose  # The pose from the message

        self.get_logger().info(f"Updating model state for {model_name} to position {pose_msg.pose.position.x}, {pose_msg.pose.position.y}")

        # Make the asynchronous service call
        future = self.client.call_async(req)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            # Get the response and check for success
            response = future.result()
            if response.success:
                self.get_logger().info("Model state updated successfully.")
            else:
                self.get_logger().warn(f"Failed to update model state: {response.message}")
        except Exception as e:
            # Catch any errors during the service call
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    # Initialize the ROS2 node and start spinning
    rclpy.init(args=args)
    node = DroneStateUpdater()
    rclpy.spin(node)

    # Cleanup and shutdown the node
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped
# from gazebo_msgs.srv import SetEntityState  # Use this import for ROS 2

# class DroneStateUpdater(Node):
#     def __init__(self):
#         super().__init__('drone_state_updater')
        
#         # Create a client for the /set_entity_state service
#         self.client = self.create_client(SetEntityState, '/set_entity_state')
        
#         # Wait for the service to be available
#         while not self.client.wait_for_service(timeout_sec=5.0):
#             self.get_logger().info('Waiting for /set_entity_state service...')
#         self.get_logger().info('Service available, starting subscriptions...')
        
#         # Subscribe to the pose topics for each drone
#         self.create_subscription(PoseStamped, '/drone_1/pose', self.drone1_callback, 10)
#         self.create_subscription(PoseStamped, '/drone_2/pose', self.drone2_callback, 10)

#     def drone1_callback(self, msg: PoseStamped):
#         self.get_logger().info(f"Received position for drone_1: {msg.pose.position.x}, {msg.pose.position.y}")
#         self.update_model_state('drone_1', msg)

#     def drone2_callback(self, msg: PoseStamped):
#         self.get_logger().info(f"Received position for drone_2: {msg.pose.position.x}, {msg.pose.position.y}")
#         self.update_model_state('drone_2', msg)

#     def update_model_state(self, model_name: str, pose_msg: PoseStamped):
#         # Create a service request to update the model state
#         req = SetEntityState.Request()
#         req.state.name = model_name  # Model name must match the name in your world
#         req.state.pose = pose_msg.pose  # The pose from the message
#         req.state.reference_frame = 'world'  # Default reference frame is 'world'

#         self.get_logger().info(f"Updating model state for {model_name} to position {pose_msg.pose.position.x}, {pose_msg.pose.position.y}")

#         # Make the asynchronous service call
#         future = self.client.call_async(req)
#         future.add_done_callback(self.handle_response)

#     def handle_response(self, future):
#         try:
#             # Get the response and check for success
#             response = future.result()
#             if response.success:
#                 self.get_logger().info("Model state updated successfully.")
#             else:
#                 self.get_logger().warn(f"Failed to update model state: {response.status_message}")
#         except Exception as e:
#             # Catch any errors during the service call
#             self.get_logger().error(f"Service call failed: {e}")

# def main(args=None):
#     # Initialize the ROS2 node and start spinning
#     rclpy.init(args=args)
#     node = DroneStateUpdater()
#     rclpy.spin(node)
    
#     # Cleanup and shutdown the node
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()