#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import demo_pkg.sonar_io as SIO


def main():
    rospy.init_node('sonar_demo', anonymous=True)
    s = SIO.SonarIO()
    
    for i in range(0, 100):
        s.set_sonars(i)
        print("State :", len(s.state()), "Enabled: ", bin(s.get_sonars()))
        rospy.sleep(1)

if __name__ == '__main__':
    main()
