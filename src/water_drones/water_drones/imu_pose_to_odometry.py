import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R  # For quaternion rotation

class ImuPoseToOdometry(Node):
    def __init__(self):
        super().__init__('imu_pose_to_odometry')

        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu, '/world/waves/model/wow_craft/link/imu_link/sensor/imu_sensor/imu', self.imu_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseArray, '/world/waves/dynamic_pose/info', self.pose_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Variables to store the latest messages
        self.latest_imu = None
        self.latest_pose = None
        self.previous_pose = None
        self.previous_time = None

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg
        
    def pose_callback(self, msg: PoseArray):
        if msg.poses:
            # Ensure the correct pose is extracted (assuming first is correct)
            self.latest_pose = msg.poses[0]
        else:
            self.get_logger().warn("PoseArray is empty!")
        self.publish_odometry()

    def publish_odometry(self):
        if self.latest_imu is None or self.latest_pose is None:
            return  # Wait until both messages are received

        current_time = self.get_clock().now()


        # Compute velocity if previous pose is available
        if self.previous_pose is not None and self.previous_time is not None:
            dt = (current_time.nanoseconds - self.previous_time.nanoseconds) * 1e-9  # Convert to seconds
            
            if dt > 0:
                vel_world = np.array([
                    (self.latest_pose.position.x - self.previous_pose.position.x) / dt,
                    (self.latest_pose.position.y - self.previous_pose.position.y) / dt,
                    (self.latest_pose.position.z - self.previous_pose.position.z) / dt
                ])
                

            else:
                self.get_logger().warn("dt is zero, skipping velocity update!")
                return
        else:
            # If no previous data, initialize velocity as zero
            vel_world = np.array([0.0, 0.0, 0.0])
            self.get_logger().warn("First update, velocity set to zero.")

        # Convert IMU quaternion to rotation matrix
        quat = self.latest_imu.orientation
        quaternion = np.array([quat.x, quat.y, quat.z, quat.w])
        rotation_matrix = R.from_quat(quaternion).as_matrix()

        # Transform velocity from world to body frame
        vel_body = rotation_matrix.T @ vel_world  # Transpose for world-to-body transformation

        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"  # Body frame of the robot

        # Set pose
        odom.pose.pose.position = self.latest_pose.position
        odom.pose.pose.orientation = self.latest_imu.orientation
        odom.pose.covariance = [0.0] * 36  # Default zero covariance

        # Set transformed velocity (in body frame)
        odom.twist.twist.linear.x = vel_body[0]
        odom.twist.twist.linear.y = vel_body[1]
        odom.twist.twist.linear.z = vel_body[2]

        # Set angular velocity from IMU (already in body frame)
        odom.twist.twist.angular.x = self.latest_imu.angular_velocity.x
        odom.twist.twist.angular.y = self.latest_imu.angular_velocity.y
        odom.twist.twist.angular.z = self.latest_imu.angular_velocity.z

        # Twist covariance
        odom.twist.covariance = [0.0] * 36

        # Publish the odometry message
        self.odom_pub.publish(odom)

        # Update previous pose and time for next iteration
        self.previous_pose = self.latest_pose
        self.previous_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = ImuPoseToOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R  # For quaternion rotation

class ImuPoseToOdometry(Node):
    def __init__(self):
        super().__init__('imu_pose_to_odometry')

        # Subscriptions
        self.imu_sub = self.create_subscription(
            Imu, '/world/waves/model/wow_craft/link/imu_link/sensor/imu_sensor/imu', self.imu_callback, 10)
        
        self.pose_sub = self.create_subscription(
            PoseArray, '/world/waves/dynamic_pose/info', self.pose_callback, 10)

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Variables to store the latest messages
        self.latest_imu = None
        self.latest_pose = None
        self.previous_pose = None
        self.previous_time = None

        # Estimated velocity in body frame
        self.velocity_body = np.array([0.0, 0.0, 0.0])

        # Complementary filter weight (adjust as needed)
        self.alpha = 0.7  # More weight on Pose estimation, less on IMU integration

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg


    def pose_callback(self, msg: PoseArray):
        if msg.poses:
            # Ensure the correct pose is extracted (assuming first is correct)
            self.latest_pose = msg.poses[0]
        else:
            self.get_logger().warn("PoseArray is empty!")
        self.publish_odometry()

    def publish_odometry(self):
        if self.latest_imu is None or self.latest_pose is None:
            return  # Wait until both messages are received

        current_time = self.get_clock().now()

        # Compute velocity if previous pose is available
        if self.previous_pose is not None and self.previous_time is not None:
            dt = (current_time.nanoseconds - self.previous_time.nanoseconds) * 1e-9  # Convert to seconds
            
            if dt > 0:
                vel_world = np.array([
                    (self.latest_pose.position.x - self.previous_pose.position.x) / dt,
                    (self.latest_pose.position.y - self.previous_pose.position.y) / dt,
                    (self.latest_pose.position.z - self.previous_pose.position.z) / dt
                ])
                
                self.get_logger().info(f"Δt: {dt:.6f} sec | Δx: {self.latest_pose.position.x - self.previous_pose.position.x:.4f} | "
                                    f"Δy: {self.latest_pose.position.y - self.previous_pose.position.y:.4f} | "
                                    f"Δz: {self.latest_pose.position.z - self.previous_pose.position.z:.4f}")
            else:
                self.get_logger().warn("dt is zero, skipping velocity update!")
                return
        else:
            # If no previous data, initialize velocity as zero and set a default dt
            vel_world = np.array([0.0, 0.0, 0.0])
            dt = 0.01  # Default dt if this is the first iteration
            self.get_logger().warn("First update, velocity set to zero.")

        # Convert IMU quaternion to rotation matrix
        quat = self.latest_imu.orientation
        quaternion = np.array([quat.x, quat.y, quat.z, quat.w])
        rotation_matrix = R.from_quat(quaternion).as_matrix()

        # Transform velocity from world to body frame
        vel_body_pose = rotation_matrix.T @ vel_world  # Transpose for world-to-body transformation

        # IMU acceleration is already in body frame
        accel_body = np.array([
            self.latest_imu.linear_acceleration.x,
            self.latest_imu.linear_acceleration.y,
            self.latest_imu.linear_acceleration.z
        ])

        # Integrate acceleration to estimate velocity from IMU
        if self.previous_time is not None:  # Skip integration on the first run
            vel_body_imu = self.velocity_body + accel_body * dt  # v_new = v_old + a * dt
        else:
            vel_body_imu = np.array([0.0, 0.0, 0.0])  # No integration on first loop

        # Apply complementary filter to combine Pose and IMU estimates
        self.velocity_body = self.alpha * vel_body_pose + (1 - self.alpha) * vel_body_imu

        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"  # Body frame of the robot

        # Set pose
        odom.pose.pose.position = self.latest_pose.position
        odom.pose.pose.orientation = self.latest_imu.orientation
        odom.pose.covariance = [0.0] * 36  # Default zero covariance

        # Set transformed velocity (in body frame)
        odom.twist.twist.linear.x = self.velocity_body[0]
        odom.twist.twist.linear.y = self.velocity_body[1]
        odom.twist.twist.linear.z = self.velocity_body[2]

        # Set angular velocity from IMU (already in body frame)
        odom.twist.twist.angular.x = self.latest_imu.angular_velocity.x
        odom.twist.twist.angular.y = self.latest_imu.angular_velocity.y
        odom.twist.twist.angular.z = self.latest_imu.angular_velocity.z

        # Twist covariance
        odom.twist.covariance = [0.0] * 36

        self.odom_pub.publish(odom)

        self.get_logger().info(f"Published velocity (body frame): X: {self.velocity_body[0]:.4f}, "
                            f"Y: {self.velocity_body[1]:.4f}, "
                            f"Z: {self.velocity_body[2]:.4f}")

        # Update previous pose and time for next iteration
        self.previous_pose = self.latest_pose
        self.previous_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = ImuPoseToOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()"""