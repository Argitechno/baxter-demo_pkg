#!/usr/bin/env python 
#   
"""
Script to test joints
"""
import numpy as np
PI = np.pi
e0 = 0
e1 = PI/2.5
s0 = PI/3.6
s1 = PI/3.6
w0 = 0
w1 = PI/2.8
w2 = 0

# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

# initialize our ROS node, registering it with the Master
rospy.init_node('test_joints')

# create instance of baxter_interface's Limb class
limb_left = baxter_interface.Limb('left')
limb_right = baxter_interface.Limb('right')

# store the joint positions
left = {
	'left_e0':  e0,
	'left_e1':  e1,
	'left_s0': -s0,
	'left_s1': -s1,
	'left_w0':  w0,
	'left_w1':  w1,
	'left_w2':  w2,
}

right = {
	'right_e0':  e0,
	'right_e1':  e1,
	'right_s0':  s0,
	'right_s1': -s1,
	'right_w0':  w0,
	'right_w1':  w1,
	'right_w2':  w2,
}


# move both arms to home position
limb_left.move_to_joint_positions(left)
limb_right.move_to_joint_positions(right)
rospy.sleep(3)
e0 = PI/2
e1 = PI/2
s0 = PI/4
s1 = PI/3
w1 = PI/2
right = {
	'right_e0':  -e0,
	'right_e1':  e1,
	'right_s0':  s0,
	'right_s1': -s1,
	'right_w1':  -w1,
}
limb_right.move_to_joint_positions(right)
rospy.sleep(3)
limb_left.move_to_neutral()
limb_right.move_to_neutral()
rospy.sleep(3)
e0 = 4*PI/10
e1 = 6*PI/10
s0 = 0*PI/10
s1 = 3*PI/10
w0 = 2*PI/10
w1 = 3*PI/10
w2 = 2*PI/10
left = {
	'left_e0': -e0,
	'left_e1':  e1,
	'left_s0':  s0,
	'left_s1': -s1,
	'left_w0':  w0,
	'left_w1':  w1,
	'left_w2': -w2,
}

right = {
	'right_e0':  e0,
	'right_e1':  e1,
	'right_s0':  s0,
	'right_s1': -s1,
	'right_w0': -w0,
	'right_w1':  w1,
	'right_w2':  w2,
}
limb_left.move_to_joint_positions(left)
limb_right.move_to_joint_positions(right)

quit()
