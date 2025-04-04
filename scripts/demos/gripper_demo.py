#!/usr/bin/env python
# license removed for brevity
import baxter_interface.cfg
import rospy
import baxter_interface


def gripper_demo():
    """Access the gripper and move it, and tell us the pose of the endpoint (where the gripper is)"""
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

def main():
    """Demonstrate usage of the gripper."""

    print("Initializing node...")
    rospy.init_node('gripper_demo')
    print("Getting robot state...")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    
    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    
    rospy.on_shutdown(clean_shutdown)
    gripper_demo()
    

if __name__ == '__main__':
    main()


