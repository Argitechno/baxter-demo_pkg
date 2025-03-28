#!/usr/bin/env python
# license removed for brevity
import rospy
import baxter_interface

def main():
        gripper = baxter_interface.Gripper('right')
        print("Gripper Demo!")
        gripper.calibrate()
        # Move to positions 0, 100, 75, 50, 25, 0
        for i in range(5, -1, -1):
            gripper.command_position((i % 5) * 25)
            rospy.sleep(3)

if __name__ == '__main__':
    rospy.init_node('test_gripper')


