#!/usr/bin/env python3

# Import necessary ROS and system libraries
import moveit_commander  # Interface for MoveIt motion planning
import rospy              # ROS Python API
import sys                # Access to system-level parameters
from geometry_msgs.msg import Pose, PoseStamped  # ROS message types for poses
import yaml               # For reading YAML config files
import armpy.arm          # Custom module to control robot arm
import armpy.gripper      # Custom module to control gripper
import traceback          # For detailed error logging
import os                 # File path operations
import kinova_msgs.srv   

class Autonomous:
    """
    Autonomous robot control class that initializes and moves a robotic arm
    to a predefined joint configuration loaded from a YAML file.
    """
    def __init__(self, goal="goal1"):
        """
        Constructor: Initializes the MoveIt commander, gripper, arm, and loads
        the target joint configuration from a YAML file.

        Args:
            goal (str): The name of the goal position in the YAML file.
        """
        try:
            print("MOVING TO GOAL ", goal)

            # Initialize MoveIt commander with system arguments
            moveit_commander.roscpp_initialize(sys.argv)

            # Initialize gripper and arm interfaces from armpy
            self.gripper = armpy.gripper.Gripper()
            self.arm = armpy.arm.Arm()

            # Set a default movement velocity for the robot arm
            self.arm.set_velocity(0.3)

            # Define the path to the joint state YAML file
            path = os.path.join(
                os.path.dirname(os.path.abspath(__file__)),  # Current script directory
                "../config/joint_states.yaml"                # Relative path to config
            )

            # Open and parse the YAML file for the specified goal
            with open(path, 'r') as file:
                self.goal = yaml.safe_load(file)[goal]       # Load joint values for the goal

            # Extract the first 7 joint positions (typical for 7-DOF arms)
            self.pos = self.goal['position'][0:7]
            print("position loaded")

        except:
            # Print detailed error traceback in case of failure
            traceback.print_exc()
            pass

    def move(self):
        """
        Plans and executes a movement of the robotic arm to the preloaded
        joint position.
        """
        # Generate a motion plan for the target joint positions
        plan = self.arm.plan_joint_pos(self.pos)

        # If a valid plan is returned, execute it
        if plan != None:
            self.arm.move_robot(plan, True)



if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('autonomous_robot', anonymous=True)

    # Create the controller 
    # Specify the goal position
    auton = Autonomous(goal="goal1")

    auton.move()

    # Keep the node alive
    rospy.spin()