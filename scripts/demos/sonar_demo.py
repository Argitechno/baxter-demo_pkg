#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import sonar_io as SIO

rospy.init_node('sonar_demo', anonymous=True)

s = SIO.SonarIO
for i in range(0, 100):
    print(len(s.state()))
    rospy.sleep(0.2)