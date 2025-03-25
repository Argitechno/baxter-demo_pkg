#!/usr/bin/env python
# liscence goes here
import rospy
import math
from sensor_msgs.msg import Imu

def callback(data):
	n = callback.n
	#We can do anything with the data here.
	xAcc = data.linear_acceleration.x
	yAcc = data.linear_acceleration.y
	zAcc = data.linear_acceleration.z
	grav = math.sqrt(xAcc * xAcc + yAcc * yAcc + zAcc * zAcc)
	if(callback.counter < n):
		callback.graverage[callback.counter] = grav;
		callback.counter = callback.counter + 1
	elif(callback.counter < 2 * n):
		idx = (callback.counter - n)%n
		callback.graverage[idx]= grav;
		callback.counter = callback.counter + 1
	else:
		print("Gravity According to data if arm is still: ")
		avgrav = 0
		for i in range(n):
			avgrav = avgrav + callback.graverage[i]
		avgrav = avgrav/n
		print(avgrav)
		callback.counter = n
callback.n = 100
callback.counter = 0
callback.graverage = [[] for k in range(callback.n)]
   
def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('accelerometer_listener', anonymous=True)
	
	#We are subscribing to the accelerometer, and will run the callback when we get a msg
	#With the argument being the msg recieved, a sensor_msgs/Imu in this case
	rospy.Subscriber("/robot/accelerometer/right_accelerometer/state", Imu, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()

