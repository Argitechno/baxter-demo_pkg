#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
import baxter_interface.digital_io as DIO
import demo_pkg.sonar_lights_io as SLIO


def _switchState(digitalComponent):
			digitalComponent.state = not digitalComponent.state

def main():
	rospy.init_node('lights_demo', anonymous=True)
	l = SLIO.SonarLightsIO()

	leftLightInner = DIO.DigitalIO('left_inner_light')
	leftLightOuter = DIO.DigitalIO('left_outer_light')
	rightLightInner = DIO.DigitalIO('right_inner_light')
	rightLightOuter = DIO.DigitalIO('right_outer_light')
	torsoLeftLightInner = DIO.DigitalIO('torso_left_inner_light')
	torsoLeftLightOuter = DIO.DigitalIO('torso_left_outer_light')
	torsoRightLightInner = DIO.DigitalIO('torso_right_inner_light')
	torsoRightLightOuter = DIO.DigitalIO('torso_right_outer_light')
	headRedLight = DIO.DigitalIO('head_red_light')
	headYellowLight = DIO.DigitalIO('head_yellow_light')
	headGreenLight = DIO.DigitalIO('head_green_light')

	def switchLights():
		_switchState(leftLightInner)
		_switchState(leftLightOuter)
		_switchState(rightLightInner)
		_switchState(rightLightOuter)
		_switchState(torsoLeftLightInner)
		_switchState(torsoLeftLightOuter)
		_switchState(torsoRightLightInner)
		_switchState(torsoRightLightOuter)
	

	for i in range(0, 10):
		switchLights()
		print("Switched the lights!", i)
		rospy.sleep(1)
	
	print("Lights Test...")
	for i in range(0, 12):
		r = (100*math.cos(math.pi * i/22))
		g = (100*math.sin(math.pi * i/22))
    	#print("Lights On: ", bin(l.get_lights()))
    	#print("Red desired: %d, Green desired: %d" % (r,g,))
		l.set_red_level(r, 4)
		l.set_green_level(g, 4)
    	#print("Red actual:  %d, Green actual:  %d" % (l.get_red_level(),l.get_green_level(),))
		rospy.sleep(0.1)

	print("Lights Show!!!")
	rate = rospy.Rate(100)
	targetExit = rospy.Time.now() + rospy.Duration(5)
	while rospy.Time.now() < targetExit:
		time_elapsed = (targetExit - rospy.Time.now())
		parts_elapsed = math.modf(3 * time_elapsed/rospy.Duration(1))[1]
		if  parts_elapsed % 2:
			l.set_lights(int('0b101010101010',2))
		else:
			l.set_lights(int('0b010101010101',2))
		rate.sleep()



if __name__ == '__main__':
    main()
