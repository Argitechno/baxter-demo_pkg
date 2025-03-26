#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import baxter_interface.digital_io as DIO





def talker():
	rospy.init_node('lights_talker')
	leftLightInner = DIO.DigitalIO('left_inner_light')
	leftLightOuter = DIO.DigitalIO('left_outer_light')
	rightLightInner = DIO.DigitalIO('right_inner_light')
	righLightOuter = DIO.DigitalIO('right_outer_light')
	torsoLeftLightInner = DIO.DigitalIO('torso_left_inner_light')
	torsoLeftLightOuter = DIO.DigitalIO('torso_left_outer_light')
	torsoRightLightInner = DIO.DigitalIO('torso_right_inner_light')
	torsoRightLightOuter = DIO.DigitalIO('torso_right_outer_light')
	def switchLights():
		def switchState(digitalComponent):
			digitalComponent.state = not digitalComponent.state
		switchState(leftLightInner)
		switchState(leftLightOuter)
		switchState(rightLightInner)
		switchState(righLightOuter)
		switchState(torsoLeftLightInner)
		switchState(torsoLeftLightOuter)
		switchState(torsoRightLightInner)
		switchState(torsoRightLightOuter)
	for i in range(0, 10):
		leftLightInner = not leftLightInner
		print("Switched the lights!", i)
		rospy.sleep(1)

	

if __name__ == '__main__':

	try:
		talker()

	except rospy.ROSInterruptException:
		pass
