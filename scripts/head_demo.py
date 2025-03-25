#!/usr/bin/env python 
#   
"""
Script to test head
"""

# rospy - ROS Python API
import rospy

# baxter_interface - Baxter Python API
import baxter_interface

# initialize our ROS node, registering it with the Master
rospy.init_node('test_head')

# create instance of baxter_interface's Limb class
head = baxter_interface.Head()
#Look left slow
head.set_pan(0.5, speed=0.05)
rospy.sleep(0.2)
#Look right FAST
head.set_pan(-0.5, speed=1.0)
rospy.sleep(0.5)
#Center slow
head.set_pan(0.0, speed=0.05)
#Nod
head.command_nod();



quit()
