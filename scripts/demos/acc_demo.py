#!/usr/bin/env python
# license removed for brevity
import rospy
import demo_pkg.accelerometer_io as AccelerometerIO

def main():
    print("Initializing node...")
    rospy.init_node('acc_demo', anonymous=True)

    print("Initializing Acc Object")
    rightAcc = AccelerometerIO.AccIO('right_accelerometer')

    print("Starting acc monitor...")
    rate = rospy.Rate(2)
    targetExit = rospy.Time.now() + rospy.Duration(5)
    while rospy.Time.now() < targetExit:
        rate.sleep()

if __name__ == '__main__':
    main()
