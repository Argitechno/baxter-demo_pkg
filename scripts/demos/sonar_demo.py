#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
import demo_pkg.sonar_io as SIO
import demo_pkg.sonar_lights_io as SLIO


def main():
    rospy.init_node('sonar_demo', anonymous=True)
    s = SIO.SonarIO()
    l = SLIO.SonarLightsIO()
    for i in range(0, 12):
        s.set_sonars(2**i)
        print("State :", len(s.state()), "Enabled: ", bin(s.get_sonars()))
        rospy.sleep(1)
    
    s.set_sonars(4095)
    print("Lights Test...")
    for i in range(0, 12):
        r = (50*math.cos(2 * math.pi * i/11) + 50)
        g = (50*math.sin(2 * math.pi * i/11) + 50)
        print("Lights On: ", bin(l.get_lights()))
        print("Red desired: %d, Green desired: %d" % (r,g,))
        print("Red actual:  %d, Green actual:  %d" % (l.get_red_level(),l.get_green_level(),))
        l.set_red_level(r)
        l.set_green_level(g)
        rospy.sleep(1)

    print("Lights Show!!!")
    rate = rospy.Rate(100)
    targetExit = rospy.Time.now() + rospy.Duration(5)
    while rospy.Time.now() < targetExit:
        l.set_lights(int('0b101010101010',2))
        rate.sleep()
        
    

if __name__ == '__main__':
    main()
