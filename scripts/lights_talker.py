#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import baxter_interface.analog_io as AIO

def talker():
	#pub = rospy.Publisher('demo_chatter', String, queue_size=10)
	rospy.init_node('lights_talker')
	rate = rospy.Rate(10) # 10hz
	#10 times a second, publish a String msg.
	torso_lighting = AIO.AnalogIO('torso_lighting')

	time_last = rospy.get_time()
	time_past = 0
	torso_lights = False
	while not rospy.is_shutdown():
		
		time_past = time_past + rospy.get_time()
		seconds_past = time_past/10000000000
		if not torso_lights and seconds_past > 3:
			torso_lights = True
			torso_lighting.set_output(100);
		if seconds_past > 10:
			torso_lighting.set_output(0);
			return
		print torso_lighting.state()
		rate.sleep()

if __name__ == '__main__':

	try:
		talker()

	except rospy.ROSInterruptException:
		pass
