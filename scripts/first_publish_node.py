#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('demo_chatter', String, queue_size=10)
	rospy.init_node('demo_talker')
	rate = rospy.Rate(10) # 10hz
	#10 times a second, publish a String msg.

	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		rospy.loginfo(hello_str)
		pub.publish(hello_str)
		rate.sleep()

if __name__ == '__main__':

	try:
		talker()

	except rospy.ROSInterruptException:
		pass
