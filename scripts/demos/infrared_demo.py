#!/usr/bin/env python
# license removed for brevity
import rospy
import baxter_interface.analog_io as AIO


def main():
    print("Initializing node...")
    rospy.init_node('infrared_demo', anonymous=True)

    print("Initializing Sensor Object")
    irs = AIO.AnalogIO('right_hand_range')

    print("Starting range monitor...")
    rate = rospy.Rate(2)
    targetExit = rospy.Time.now() + rospy.Duration(5)
    while rospy.Time.now() < targetExit:
        print(irs.state())
        rate.sleep()

if __name__ == '__main__':
    main()
