#!/usr/bin/env python
# license removed for brevity
import baxter_interface.cfg
import rospy
import baxter_interface

def main():
    print("Initializing node...")
    rospy.init_node('gripper_demo')
    gripper = baxter_interface.Gripper('right')
    right_limb = baxter_interface.Limb('right')
    print("Gripper Demo!")

    print("Current Right Wrist Pose: ")
    print(right_limb.endpoint_pose())


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


