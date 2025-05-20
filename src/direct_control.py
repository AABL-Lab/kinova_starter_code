#!/usr/bin/env python3

import rospy
import armpy.gen2_teleop
from teleop_lib.gui.teleop_config_frame import TeleopConfigFrame, get_teleop_info
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
import traceback
import yaml
import teleop_lib.input_profile
from sensor_msgs.msg import Joy
import os
import armpy.gripper
import kinova_msgs.srv

class Direct_Control:
    """
    Direct_Control handles joystick-based teleoperation of the Kinova Gen2 robotic arm,
    including arm velocity control and gripper actuation.
    """

    def __init__(self, homing=True):

        # Load joystick-to-arm mapping configuration from YAML
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/XYZMode.yaml")
        with open(path, 'r') as file:
            self.cfg = yaml.safe_load(file)

        # Placeholder for current command
        self.cmd = None

        # Initialize gripper control interface
        self.gripper = armpy.gripper.Gripper()
        
        # Build joystick input profile based on configuration
        self.mode = teleop_lib.input_profile.build_profile(self.cfg)

        # Placeholder for current pose
        self.pose = PoseStamped()
        
        # Initialize arm control interface (no homing on startup)
        self.arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver", home_arm=homing)

    def start(self):
        """
        Initializes ROS subscriber for joystick.
        """
        self.joy_subscriber = rospy.Subscriber("/joy", Joy, self.callback)

    def callback(self, data):
        """
        Main joystick callback to process input and control the arm.

        Args:
            data (Joy): Joystick input message.
        """

        # Get the processed twist command from joystick input
        command = self.mode.process_input(data).twist

        # Create a new Twist message for linear velocity
        new_cmd = Twist()
        new_cmd.linear.x = command.linear.x
        new_cmd.linear.y = command.linear.y
        new_cmd.linear.z = command.linear.z

        # Send velocity command to the robotic arm
        self.arm.set_velocity(new_cmd)

    def open_gripper(self):
        """
        Opens the robot gripper.
        """
        self.gripper.open()

    def close_gripper(self):
        """
        Closes the robot gripper.
        """
        self.gripper.close()

    def stop_arm(self):
        """
        Stops all motion of the robotic arm.
        """
        self.arm.stop()

    def home_arm(self):
        """
        Calls the home service to bring the arm to its home position.
        """
        rospy.wait_for_service("/j2s7s300_driver/in/home_arm")
        home_arm = rospy.ServiceProxy("/j2s7s300_driver/in/home_arm", kinova_msgs.srv.HomeArm)
        home_arm()

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('robot_xbox_controller', anonymous=True)

    # Create the controller 
    # Home the arm at startup
    xbox = Direct_Control(homing=True)

    # Start listening to joystick 
    xbox.start()

    # Keep the node alive
    rospy.spin()
