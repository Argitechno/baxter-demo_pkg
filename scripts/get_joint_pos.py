#!/usr/bin/env python 
#   
"""
get_joint_pos.py

Simple ROS node that prints the current joint angles of Baxter's left and right arms.
Useful for debugging.
"""
import rospy
import baxter_interface

def main():
	print("Initializing ROS node...")
	rospy.init_node('get_joint_pos')

	print("Create instance of baxter_interface's Limb class...")
	limb_left = baxter_interface.Limb('left')
	limb_right = baxter_interface.Limb('right')

	print("Printing current joint angles...")
	print("Left Position:")
	print(limb_left.joint_angles())
	print()
	print("Right Position:")
	print(limb_right.joint_angles())

if __name__ == '__main__':
	main()