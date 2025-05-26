# Imports
import sys
import argparse
import time  
from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

# You may add any other imports you may need/want to use below
# import ...


class decision_maker(Node):
    
    #def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER):
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, rate=10, motion_type=POINT_PLANNER):
        super().__init__("decision_maker")

        #TODO Part 4: Create a publisher for the topic responsible for robot's motion
        self.publisher = self.create_publisher(publisher_msg, publishing_topic, qos_publisher)


        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(POINT_PLANNER)    
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(TRAJECTORY_PLANNER)

        else:
            print("Error! you don't have this planner", file=sys.stderr)
  
        self.localizer = localization(rawSensor)
       
        self.goal=self.planner.plan()
        print("self.goal", type(self.goal))
        print(self.goal)
        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        # TODO Part 3: Run the localization node
        # Remember that this file is already running the decision_maker node.
        # Run the localization node by checking and updating the current pose
        spin_once(self.localizer)
        current_pose = self.localizer.getPose()
        if  current_pose is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # TODO Part 3: Check if you reached the goal
        if type(self.goal) == list:  # Trajectory

            if calculate_linear_error(current_pose, self.goal[-1]) < 0.1:
                reached_goal= True
            else: 
                reached_goal = False

        else:  # Point
            if calculate_linear_error(current_pose, self.goal) < 0.1:
                reached_goal= True
            else: 
                reached_goal = False


        if reached_goal:
            print("reached goal")
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.publisher.publish(vel_msg) # Publish zero velocity to stop the robot
            if hasattr(self.controller, 'logger'):  # Ensure the controller has a logger
                self.controller.logger.save_log() # Save logs for the single controller
            else:
                print("No logger found in the controller.")
            #TODO Part 3: exit the spin
            # Stop the node and exit the program
            self.destroy_node()  # Shutdown the node
            print("Node shutdown. Exiting program.")
            sys.exit(0)  # Exit the program
        
        velocity, yaw_rate = self.controller.vel_request(current_pose, self.goal, True)
        print("velocity ", velocity)
        print("yaw rate", yaw_rate)

        #TODO Part 4: Publish the velocity to move the robot
        vel_msg.linear.x = velocity
        vel_msg.angular.z = yaw_rate
        self.publisher.publish(vel_msg)




def main(args=None):
    
    init()

    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    odom_qos=QoSProfile(reliability=2,durability=2, history=1, depth=10)
    

    # TODO Part 4: instantiate the decision_maker with the proper parameters for moving the robot
    if args.motion.lower() == "point":
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, rate=10, motion_type=POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", odom_qos, rate=10, motion_type=TRAJECTORY_PLANNER)
    else:
        print("invalid motion type", file=sys.stderr)        
        return
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point", help="Type of motion: 'point' or 'trajectory'")
    args = argParser.parse_args()

    main(args)
