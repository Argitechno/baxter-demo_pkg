#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import demo_pkg.sonar_io as SIO


def main():
    rospy.init_node('sonar_demo', anonymous=True)
    s = SIO.SonarIO()
    for i in range(0, 100):
        print(len(s.state()))
        rospy.sleep(0.2)

if __name__ == '__main__':
    main()
