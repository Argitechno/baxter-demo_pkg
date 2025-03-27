#!/usr/bin/env python
# license removed for brevity
import rospy
import demo_pkg.sonar_io as SIO


def main():
    rospy.init_node('sonar_demo', anonymous=True)
    s = SIO.SonarIO()
    for i in range(0, 12):
        s.set_sonars(2**i)
        print("State :", len(s.state()), "Enabled: ", bin(s.get_sonars()))
        rospy.sleep(1)
    s.set_sonars(4095)

if __name__ == '__main__':
    main()
