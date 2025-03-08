import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time
import numpy as np

class PurePursuitControllerWithTiming:
    def __init__(self, lookahead_distance=0.6, speed=0.3,goal_tolerance=0.4):
        self.lookahead_distance = lookahead_distance
        self.speed = speed
        self.waypoints = []
        self.timestamps = []
        self.goal_tolerance = goal_tolerance
        self.current_waypoint_index = 0
        self.goal_reached = False
        self.p = 0.01
        self.d = 0.01
        self.start_time = time.time()
        self.prev_time = time.time()
        self.prev_time_error = 0
        self.time_index = 0


    def set_waypoints(self, waypoints):
        self.waypoints = np.array(waypoints)
        self.current_waypoint_index = 0
        self.goal_reached = False

    def set_timestamps(self, timestamps):
        self.timestamps = 2*timestamps #avoid out of index

    def compute_control(self, odometry_msg, logger):
        if self.goal_reached:
            return Twist()

        # Extract current pose
        position = odometry_msg.pose.pose.position
        orientation = odometry_msg.pose.pose.orientation
        
        # Calculate current yaw
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
        )
        current_pose = (position.x, position.y)

        # Check if all waypoints are reached
        if self.current_waypoint_index >= len(self.waypoints):
            self.goal_reached = True
            logger.info("Goal reached!")
            return Twist()
        
        goal_point = self.waypoints[self.current_waypoint_index]
        
        while (self.current_waypoint_index < len(self.waypoints) - 1 and np.linalg.norm(current_pose - self.waypoints[self.current_waypoint_index + 1]) <= self.lookahead_distance):
            goal_point = self.waypoints[self.current_waypoint_index+1, : ]
            self.current_waypoint_index += 1

        goal_x, goal_y = goal_point[:2]

        # Log current position and goal
        logger.info(f"Current position: x={current_pose[0]:.2f}, y={current_pose[1]:.2f}")
        logger.info(f"Current goal: x={goal_x:.2f}, y={goal_y:.2f}")
        logger.info(f"Yaw: {yaw}")

        # Calculate distances
        distance_to_goal = math.hypot(goal_x - current_pose[0], goal_y - current_pose[1])

        if distance_to_goal <= self.goal_tolerance:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.goal_reached = True
                logger.info("Goal reached!")
                return Twist()

        current_time = time.time() - self.start_time
        dt = time.time() - self.prev_time

        #Check time
        while (self.timestamps[self.time_index] < current_time):
            self.time_index += 1

        #Adjust speed according to time
        time_error = current_time - self.timestamps[self.time_index]
        proportional = -self.p * time_error
        derivative = -(self.d * (time_error - self.prev_time_error) / dt)
        self.prev_time_error = time_error
        self.speed += proportional + derivative

        # Calculate goal angle and angular velocity
        goal_angle = math.atan2(goal_y - current_pose[1], goal_x - current_pose[0])
        angular_difference = (goal_angle - yaw + math.pi) % (2 * math.pi) - math.pi
        logger.info(f"Angular difference: {angular_difference}")
        #angular_velocity = 0.5 * angular_difference


        # Create twist message
        twist_msg = Twist()
        twist_msg.linear.x = min(self.speed, distance_to_goal)*math.cos(angular_difference)
        twist_msg.linear.y = min(self.speed, distance_to_goal)*math.sin(angular_difference)
        #twist_msg.angular.z = angular_velocity

        return twist_msg

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.controller = PurePursuitControllerWithTiming()
        
        waypoints = [
                    (1.0, 1.0),
                    (2.0, 2.0),
                    (2.0, 3.0),
                    (2.0, 4.0),
                    (2.0, 5.0),
                    (1.5, 5.5),
                    (1.0, 6.0),
                    (0.0, 6.5),
                    (-1.0, 6.0),
                    (-1.5, 5.5),
                    (-2.0, 5.0),
                    (-2.0, 4.0),
                    (-2.0, 3.0),
                    (-2.0, 2.0),
                    (-1.0, 1.0),
                    (1.0, -1.0),
                    (2.0, -2.0),
                    (2.0, -3.0),
                    (2.0, -4.0),
                    (2.0, -5.0),
                    (1.5, -5.5),
                    (1.0, -6.0),
                    (0.0, -7.0),
                    (-1.0, -6.0),
                    (-1.5, -5.5),
                    (-2.0, -5.0),
                    (-2.0, -4.0),
                    (-2.0, -3.0),
                    (-2.0, -2.0),
                    (-1.0, -1.0),
                    (0.0, 0.0)]

        # Example waypoints without timestamps
        self.controller.set_waypoints(waypoints)

        self.controller.set_timestamps([time for time in range(len(waypoints))])

        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def odom_callback(self, odometry_msg):
        twist_msg = self.controller.compute_control(odometry_msg, self.get_logger())
        self.cmd_vel_publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()