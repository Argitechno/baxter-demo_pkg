#!/usr/bin/env python
# license removed for brevity
import argparse
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import demo_pkg.accelerometer_io as AccelerometerIO

def acc_monitor():
    """Monitor the accelerometer for 5 seconds."""

    print("Initializing Acc Object")
    rightAcc = AccelerometerIO.AccelerometerIO('right_accelerometer')
    print("Starting acc monitor...")
    rate = rospy.Rate(2)
    targetExit = rospy.Time.now() + rospy.Duration(5)
    while rospy.Time.now() < targetExit:
        print('Linear Acc: ')
        print(rightAcc.get_linear_acceleration())
        print('\n')
        rate.sleep()

def main():
    """Demonstrate that the accellerometers are accessible."""

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(   formatter_class = arg_fmt,
                                        description = main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node...")
    rospy.init_node('acc_demo', anonymous=True)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)
    acc_monitor()

if __name__ == '__main__':
    main()