#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import baxter_interface.digital_io as DIO

leftLightInner = DIO.DigitalIO('left_inner_light')
leftLightOuter = DIO.DigitalIO('left_outer_light')
rightLightInner = DIO.DigitalIO('right_inner_light')
righLightOuter = DIO.DigitalIO('right_outer_light')
torsoLeftLightInner = DIO.DigitalIO('torso_left_inner_light')
torsoLeftLightOuter = DIO.DigitalIO('torso_left_outer_light')
torsoRightLightInner = DIO.DigitalIO('torso_right_inner_light')
torsoRightLightOuter = DIO.DigitalIO('torso_right_outer_light')
headRedLight = DIO.DigitalIO('head_red_light')
headYellowLight = DIO.DigitalIO('head_yellow_light')
headGreenLight = DIO.DigitalIO('head_green_light')
def switchState(digitalComponent):
		digitalComponent.state = not digitalComponent.state
def switchLights():
	switchState(leftLightInner)
	switchState(leftLightOuter)
	switchState(rightLightInner)
	switchState(righLightOuter)
	switchState(torsoLeftLightInner)
	switchState(torsoLeftLightOuter)
	switchState(torsoRightLightInner)
	switchState(torsoRightLightOuter)
for i in range(0, 10):
	switchLights()
	print("Switched the lights!", i)
	rospy.sleep(1)
print(headRedLight.state, " : HRL")
headRedLight.state = not headRedLight
print(headRedLight.state, " : HRL")
