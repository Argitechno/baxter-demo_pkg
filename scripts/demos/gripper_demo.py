#!/usr/bin/env python
# license removed for brevity
import rospy
import baxter_interface

def main():
    print("Initializing node...")
    rospy.init_node('gripper_demo')
    gripper = baxter_interface.Gripper('right')
    print("Gripper Demo!")
    gripper.calibrate()
    gripper.close()
    rospy.sleep(3)
    # Move to positions 100, 75, 50, 25, 0
    for i in range(4, -1, -1):
        gripper.command_position(i  * 25)
        rospy.sleep(3)
    gripper.open()

if __name__ == '__main__':
    main()


