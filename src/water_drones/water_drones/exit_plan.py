import numpy as np
import casadi as ca
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry






#############################################################################################
# Steps to do
# 
# - Define the area of the choreography
# - Define exit-spots, based on the area and the number of drones
# - Create a ros node than can subscribe to the actual position of the drones
# - Based on the actual position, define a function that:
#       - selects the exit-spot that is the closest to each drone
#       - Computes the path to this spot
#       - Does collision avoidance
# - As soon as the user wants to execute the exit-plan, the other file has to be stopped
# - New position will be published to the drones
#############################################################################################



# Parameters
number_of_drones = 20
length = 6 # at this side the drone will be placed with equal space
width = 6


def get_spots():
    rest = number_of_drones % 2
    if rest == 1:
        drones_side_A = int((number_of_drones - 1) / 2 + 1)
        drones_side_B = int((number_of_drones -1 ) / 2)
    else:
        drones_side_A = int(number_of_drones / 2)
        drones_side_B = drones_side_A

    exit_pos_A = np.zeros((drones_side_A,2))
    exit_pos_B = np.zeros((drones_side_B,2))

    exit_pos_A[:,0] = np.linspace(0,length,drones_side_A)
    exit_pos_B[:,0] = np.linspace(0,length,drones_side_B)
    exit_pos_B[:,1] = width 
    exit_spots = np.vstack((exit_pos_A, exit_pos_B))

    return exit_spots

def optimization(target_pos, act_pos):

    waypoints = np.zeros((1,2*number_of_drones))

    return waypoints


class ExitPlan(Node):
    def __init__(self):
        super().__init__('Exit_Plan')

        # Create a subscriber to read in the actual position of the drone
        self.pose_sub = self.create_subscription(
            PoseArray, '/world/waves/dynamic_pose/info', self.pose_callback, 10)
        
        # Create a publisher to hand over the new defined waypoints
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.latest_pose = None

    def pose_callback(self, msg: PoseArray):
        self.latest_pose = msg

    def return_latest(self):
        return self.latest_pose
    
    def compute_fastest_path(self, exit_spots, act_pos):
        """Compute the fastest path from the actual position to the exit-spots for each drone"""

        waypoints = optimization(exit_spots, act_pos)

        self.publish_odometry()

        


    def publish_odometry(self):
        """Publish the position, orientation and velocity of each drone"""


        # Create odom message
        odom = Odometry()

        self.odom_pub.publish(odom)






def main(args=None):

    spots = get_spots()
    rclpy.init(args=args)
    node = ExitPlan()
    try:
        while rclpy.ok():
            command = input("Press ENTER to get latest message (or type 'exit' to quit): ")
            if command.lower() == "exit":
                break

            rclpy.spin_once(node, timeout_sec=0.1)  # Process incoming messages
            latest_message = node.return_latest()
            if latest_message is not None:
                print(f"Latest message: {latest_message}")
                node.compute_fastest_path(spots, latest_message)
            else:
                print("No message received yet.")

    finally:
        node.destroy_node()
        rclpy.shutdown()


    
    




if __name__ == "__main__":

    main()