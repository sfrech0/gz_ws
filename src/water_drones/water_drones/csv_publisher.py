import rclpy
from rclpy.node import Node
import pandas as pd
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

class CSVPublisher(Node):
    def __init__(self):
        super().__init__('csv_publisher')

        # Load CSV file
        self.csv_data = pd.read_csv("/home/sfr/gz_ws/src/water_drones/data/drone_position.csv")

        # Initialize publishers dictionary
        self.drone_publishers = {}

        # Create a publisher for each drone
        for drone_id in self.csv_data["drone_id"].unique():
            topic_name = f"/{drone_id}/pose"
            self.drone_publishers[drone_id] = self.create_publisher(PoseStamped, topic_name, 10)
            self.get_logger().info(f"Publisher created for {topic_name}")

        self.get_logger().info("CSV Publisher Node Started!")
        self.publish_positions()

    def publish_positions(self):
        while rclpy.ok():
            start_time = time.time()

            for _, row in self.csv_data.iterrows():
                # Synchronize to the timestamp in CSV
                current_time = time.time() - start_time
                if row["timestamp"] > current_time:
                    time.sleep(row["timestamp"] - current_time)

                # Create the message to publish
                msg = PoseStamped()
                msg.header = Header()
                msg.pose = Pose()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.pose.position.x = row["x"]
                msg.pose.position.y = row["y"]
                msg.pose.position.z = 0.0  # Keep drones in 2D
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = 0.0
                msg.pose.orientation.w = 1.0

                # Publish the message
                self.drone_publishers[row["drone_id"]].publish(msg)
                self.get_logger().info(f"Published position for drone {row['drone_id']} at ({row['x']}, {row['y']})")

def main(args=None):
    rclpy.init(args=args)

    # Create the CSVPublisher node
    node = CSVPublisher()

    # Spin the node to process messages
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
