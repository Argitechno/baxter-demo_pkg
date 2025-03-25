#!/usr/bin/env python
# liscence goes here
import rospy
from std_msgs.msg import String

def callback(data):
	#We can do anything with the data here.
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   
def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('demo_listener', anonymous=True)
	
	#We are subscribing to demo_chatter, and will run the callback when we get a msg
	#With the argument being the msg recieved, a String in this case
	rospy.Subscriber("demo_chatter", String, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

if __name__ == '__main__':
	listener()

