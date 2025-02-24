import rclpy
import pandas as pd
import time
from rclpy.node import Node
from geometry_msgs.msg import Pose

class DronePositionPublisher(Node):
    def __init__(self):
        super().__init__('drone_position_publisher')

        # Read CSV file
        self.csv_data = pd.read_csv("/home/sfr/gz_ws/src/water_drones/data/drone_position.csv")
        self.get_logger().info(f"Loaded CSV file with {len(self.csv_data)} entries.")

        # Create publishers for each drone
        self.drone_publishers = {
            "drone_1": self.create_publisher(Pose, "/drone_1/pose", 10),
            "drone_2": self.create_publisher(Pose, "/drone_2/pose", 10)
        }

        # Start by reading the first timestamp
        self.current_index = 0
        self.last_publish_time = time.time()  # The time when the first position is published

        # Timer to update positions periodically (1 second)
        self.timer = self.create_timer(1.0, self.update_positions)

    def update_positions(self):
        # Ensure we don't run past the CSV data
        if self.current_index >= len(self.csv_data):
            self.get_logger().info("All positions published. Restarting from the beginning...")
            self.current_index = 0  # Restart from the beginning
            self.last_publish_time = time.time()  # Reset last published time

        # Get the next row of data
        row = self.csv_data.iloc[self.current_index]
        drone_id, x, y = row['drone_id'], row['x'], row['y']
        current_timestamp = row['timestamp']

        # Calculate the time difference since the last published position
        elapsed_time = time.time() - self.last_publish_time
        time_diff = current_timestamp - elapsed_time

        # Log the time details for debugging
        self.get_logger().info(f"Elapsed Time: {elapsed_time:.2f} | Time Diff: {time_diff:.2f} | Current Timestamp: {current_timestamp}")

        # Check if enough time has passed before publishing the next position
        if elapsed_time >= time_diff:
            # Create Pose message
            pose_msg = Pose()
            pose_msg.position.x = float(x)
            pose_msg.position.y = float(y)
            pose_msg.position.z = 0.1  # Keep drones slightly above ground

            pose_msg.orientation.x = 0.0
            pose_msg.orientation.y = 0.0
            pose_msg.orientation.z = 0.0
            pose_msg.orientation.w = 1.0

            # Publish to correct drone topic
            if drone_id == 1:
                self.drone_publishers["drone_1"].publish(pose_msg)
                self.get_logger().info(f"📡 Published pose for drone_1: (x: {x}, y: {y})")
            elif drone_id == 2:
                self.drone_publishers["drone_2"].publish(pose_msg)
                self.get_logger().info(f"📡 Published pose for drone_2: (x: {x}, y: {y})")

            # Update last published time and move to the next index
            self.last_publish_time = time.time()
            self.current_index += 1
        else:
            # Log the time until the next position update
            self.get_logger().info(f"Waiting for {time_diff - elapsed_time:.2f} seconds before publishing next position.")

def main():
    rclpy.init()
    node = DronePositionPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
