#!/usr/bin/env python 
#   
"""
Script to test joints
"""
import numpy as np
import rospy
import baxter_interface

def main():
	rospy.init_node('lights_demo', anonymous=True)

if __name__ == '__main__':
	# Initialize our ROS node, registering it with the Master
	rospy.init_node('test_joints')
	PI = np.pi
	# Create instance of baxter_interface's Limb class
	limb_left = baxter_interface.Limb('left')
	limb_right = baxter_interface.Limb('right')
	left_pos =  { 'left_e0' :  0, 'left_e1' :  PI/2.5, 'left_s0' : -PI/3.6, 'left_s1' : -PI/3.6, 'left_w0' :  0, 'left_w1' :  PI/2.8, 'left_w2' :  0}
	right_pos = { 'right_e0':  0, 'right_e1':  PI/2.5, 'right_s0':  PI/3.6, 'right_s1': -PI/3.6, 'right_w0':  0, 'right_w1':  PI/2.8, 'right_w2':  0}
	limb_left.move_to_joint_positions(left_pos)
	limb_right.move_to_joint_positions(right_pos)
	rospy.sleep(3)

	right_pos = { 'right_e0':  -PI/2, 'right_e1':  PI/2, 'right_s0':  PI/4, 'right_s1': -PI/3, 'right_w1':  -PI/2}
	limb_right.move_to_joint_positions(right_pos)
	rospy.sleep(3)

	limb_left.move_to_neutral()
	limb_right.move_to_neutral()
	rospy.sleep(3)

	left_pos =  { 'left_e0'  :  -2*PI/5, 'left_e1'  :  3*PI/5, 'left_s0'  : 0, 'left_s1'  :  -3*PI/10, 'left_w0'  :   PI/5, 'left_w1'  :  3*PI/10, 'left_w2'  :  -PI/5}
	right_pos = { 'right_e0' :   2*PI/5, 'right_e1' :  3*PI/5, 'right_s0' : 0, 'right_s1' :  -3*PI/10, 'right_w0' :  -PI/5, 'right_w1' :  3*PI/10, 'right_w2' :   PI/5}
	limb_left.move_to_joint_positions(left_pos)
	limb_right.move_to_joint_positions(right_pos)