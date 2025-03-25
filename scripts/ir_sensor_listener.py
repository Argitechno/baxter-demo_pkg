#!/usr/bin/env python
# liscence goes here
import rospy
import math
from sensor_msgs.msg import Range

def callback(data):
	#We can do anything with the data here.
	if(data.range < data.max_range and data.range > data.min_range):
		print("Range is: ", data.range)
	else:
		print("Range is out of bounds.")
		
   
def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('accelerometer_listener', anonymous=True)
	
	#We are subscribing to the accelerometer, and will run the callback when we get a msg
	#With the argument being the msg recieved, a sensor_msgs/Imu in this case
	rospy.Subscriber("/robot/range/left_hand_range/state", Range, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()

