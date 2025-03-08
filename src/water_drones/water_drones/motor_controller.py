import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Publishers for the motor thrust topics
        self.motor1_pub = self.create_publisher(Float64, '/model/wow_craft/joint/thruster_1_propeller_joint/cmd_thrust', 10)
        self.motor2_pub = self.create_publisher(Float64, '/model/wow_craft/joint/thruster_2_propeller_joint/cmd_thrust', 10)
        self.motor3_pub = self.create_publisher(Float64, '/model/wow_craft/joint/thruster_3_propeller_joint/cmd_thrust', 10)
        self.motor4_pub = self.create_publisher(Float64, '/model/wow_craft/joint/thruster_4_propeller_joint/cmd_thrust', 10)

        # Subscriber for Twist messages
        self.twist_sub = self.create_subscription(Twist, '/cmd_vel', self.twist_callback, 10)

        # Radius of the circle formed by the motors (adjust as per your watercraft)
        self.radius = 0.28  # Derived from SDF

    def twist_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = 7.5*msg.linear.x #configured to simulation
        linear_y = 7.5*msg.linear.y
        angular_z =8.276*msg.angular.z

        # Calculate motor thrusts based on velocities
        # Using superposition of linear and rotational components
        motor1_thrust = - linear_y - angular_z * self.radius
        motor2_thrust =  linear_x - angular_z * self.radius
        motor3_thrust = + linear_y - angular_z * self.radius
        motor4_thrust = - linear_x - angular_z * self.radius

        # Publish thrusts to motors
        self.publish_thrust(self.motor1_pub, motor1_thrust, "Motor 1")
        self.publish_thrust(self.motor2_pub, motor2_thrust, "Motor 2")
        self.publish_thrust(self.motor3_pub, motor3_thrust, "Motor 3")
        self.publish_thrust(self.motor4_pub, motor4_thrust, "Motor 4")

    def publish_thrust(self, publisher, thrust, motor_name):
        # Ensure thrust is within a valid range (e.g., -200 to 200)
        thrust = max(min(thrust, 1000), -1000)
        msg = Float64()
        msg.data = thrust
        publisher.publish(msg)
        #self.get_logger().info(f'{motor_name} thrust: {thrust:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()