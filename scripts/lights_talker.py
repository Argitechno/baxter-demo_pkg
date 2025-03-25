#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import baxter_interface.digital_io as DIO

def talker():
	rospy.init_node('lights_talker')
	leftLightInner = DIO.DigitalIO('left_itb_light_inner')
	
	leftLightInner.state(not leftLightInner.state())
	rospy.sleep(1)
	leftLightInner.state(not leftLightInner.state())
	rospy.sleep(1)
	leftLightInner.state(not leftLightInner.state())
	rospy.sleep(1)
	leftLightInner.state(not leftLightInner.state())
	rospy.sleep(1)
	leftLightInner.state(not leftLightInner.state())
	rospy.sleep(1)
	leftLightInner.state(not leftLightInner.state())

if __name__ == '__main__':

	try:
		talker()

	except rospy.ROSInterruptException:
		pass
