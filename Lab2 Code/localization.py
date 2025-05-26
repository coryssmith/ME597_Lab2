import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

#from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry

from rclpy import init, spin

rawSensor = 0
class localization(Node):
    
    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")
        
        # TODO Part 3: Define the QoS profile variable based on whether you are using the simulation (Turtlebot 3 Burger) or the real robot (Turtlebot 4)
        # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
        odom_qos = QoSProfile(
            reliability=2,  # Matches the topic's RELIABLE setting
            durability=2,
            history=1,   # Matches the topic's VOLATILE setting
            depth=10                                # Retains enough message history
        )
        
        self.loc_logger=Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose=None
        
        if localizationType == rawSensor:
        # TODO Part 3: subscribe to the position sensor topic (Odometry)
            self.odom_subscriber = self.create_subscription(
                Odometry, "/odom",self.odom_callback,qos_profile=odom_qos
            )
        else:
            print("This type doesn't exist", sys.stderr)
    
    
    def odom_callback(self, odom_msg):
        # TODO Part 3: Read x,y, theta, and record the stamp
        # Extract x and y position
        print("Odom message received in localizer!")  # Debug print

        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y

    # Correct quaternion extraction from orientation
        quat = [
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
        odom_msg.pose.pose.orientation.w
        ]

        theta = euler_from_quaternion(quat)  # Convert quaternion to yaw

        timestamp = odom_msg.header.stamp
    # Update current pose
        self.pose = [x, y, theta, timestamp]

    # Log data
        self.loc_logger.log_values([x, y, theta, Time.from_msg(timestamp).seconds_nanoseconds()])

    def getPose(self):
        #Retrieve the current pose of the robot.
        return self.pose

# TODO Part 3
# Here put a guard that makes the node run, ONLY when run as a main thread!
# This is to make sure this node functions right before using it in decision.py
    
if __name__ == "__main__":
    init(args=sys.argv)
    node = localization()
    try:
        spin(node)
    finally:
        node.destroy_node()

