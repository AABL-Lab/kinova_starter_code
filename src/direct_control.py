#!/usr/bin/env python3

import rospy
import armpy.gen2_teleop
from teleop_lib.gui.teleop_config_frame import TeleopConfigFrame, get_teleop_info
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
import traceback
import yaml
import teleop_lib.input_profile
from sensor_msgs.msg import Joy
import os
import armpy.gripper
import kinova_msgs.srv
                                  
                           
class Direct_Control:
    def __init__(self):

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../config/XYZMode.yaml")
        
        with open(path, 'r') as file:
            self.cfg = yaml.safe_load(file)

        self.cmd = None

        self.gripper = armpy.gripper.Gripper()
        
        self.mode = teleop_lib.input_profile.build_profile(self.cfg)

        self.pose = PoseStamped()
        
        self.arm = armpy.gen2_teleop.Gen2Teleop(ns="/j2s7s300_driver", home_arm=False)

    def check_pose(self, data):
        self.pose = data 

    def start(self):
        self.joy_subscriber = rospy.Subscriber("/joy", Joy, self.callback)
        self.pose_subscriber =rospy.Subscriber("/j2s7s300_driver/out/tool_pose", PoseStamped, self.check_pose)

    def callback(self, data):

        if data.buttons[4] == 1 and data.buttons[5] == 1: 
            self.arm.stop()
            self.open_gripper() 


        command = self.mode.process_input(data).twist
        new_cmd = Twist()
        new_cmd.linear.x = command.linear.x
        new_cmd.linear.y = command.linear.y
        new_cmd.linear.z = command.linear.z


        self.arm.set_velocity(new_cmd)


    def open_gripper(self):
        self.gripper.open()

    def close_gripper(self):
        self.gripper.close()

    def stop_arm(self):
        self.arm.stop()

    def home_arm(self):
        rospy.wait_for_service("/j2s7s300_driver/in/home_arm")
        home_arm = rospy.ServiceProxy("/j2s7s300_driver/in/home_arm", kinova_msgs.srv.HomeArm)
        home_arm()

if __name__ == "__main__":
    rospy.init_node('robot_xbox_controller', anonymous=True)

    controller = Direct_Control()
    controller.home_arm()
    controller.start()

    rospy.spin()