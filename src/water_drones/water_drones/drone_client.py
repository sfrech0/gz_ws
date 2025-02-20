import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from gz.msgs.pose_pb2 import Pose  # Gazebo message type
#from ros_gz_bridge.msg import Bridge

class GazeboRepublisher(Node):
    def __init__(self):
        super().__init__('gazebo_republisher')

        # Dictionary to store subscribers and publishers for each drone
        self.drone_publishers = {}

        # List of drone IDs (make sure it matches your CSV data)
        self.drone_ids = [1, 2]  # Adjust based on your data

        for drone_id in self.drone_ids:
            ros_topic = f"/drone_{drone_id}/pose"
            gz_topic = f"/model/drone_{drone_id}/pose"

            # Subscribe to ROS PoseStamped
            self.create_subscription(PoseStamped, ros_topic, self.pose_callback, 10)

            # Create a publisher for Gazebo
            self.drone_publishers[drone_id] = self.create_publisher(Pose, gz_topic, 10)

            self.get_logger().info(f"Bridge set up for {ros_topic} → {gz_topic}")

    def pose_callback(self, msg):
        drone_id = int(msg.header.frame_id.split('_')[-1])  # Extract drone ID

        if drone_id not in self.drone_publishers:
            self.get_logger().warn(f"Received data for unknown drone {drone_id}")
            return

        # Convert PoseStamped (ROS) to Pose (Gazebo)
        gz_msg = Pose()
        gz_msg.position.x = msg.pose.position.x
        gz_msg.position.y = msg.pose.position.y
        # gz_msg.position.z = msg.pose.position.z

        # Publish to Gazebo
        self.drone_publishers[drone_id].publish(gz_msg)

        self.get_logger().info(f"Republished pose for drone {drone_id}")

def main(args=None):
    rclpy.init(args=args)
    node = GazeboRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
