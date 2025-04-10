#!/usr/bin/env python 
#   
"""
Script to test joints
"""
import rospy
import baxter_interface

def main():
	# Initialize our ROS node, registering it with the Master
	print("Initializing node...")
	rospy.init_node('get_joint_pos')
	# Create instance of baxter_interface's Limb class
	limb_left = baxter_interface.Limb('left')
	limb_right = baxter_interface.Limb('right')
	print("Left Position: ")
	print(limb_left.joint_angles())
	print()
	print("Right Position: ")
	print(limb_right.joint_angles())

if __name__ == '__main__':
	main()