#!/usr/bin/env python 
#   
"""
Script to test gripper
"""

# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

# initialize our ROS node, registering it with the Master
rospy.init_node('test_gripper')

# create instance of baxter_interface's Limb class
gripper = baxter_interface.Gripper('right')
print("Gripper Demo!")
gripper.close()
gripper.open()
rospy.sleep(3)
gripper.command_position(75)
rospy.sleep(3)
gripper.command_position(50)
rospy.sleep(3)
gripper.command_position(0)
rospy.sleep(3)
gripper.command_position(100)
quit()
