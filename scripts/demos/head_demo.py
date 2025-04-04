#!/usr/bin/env python
# license removed for brevity
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

def head_demo():
    """Access the head and pan + nod it"""

    print("Begin scan")
    head = baxter_interface.Head()
    head.set_pan(0.5, speed=0.05)
    rospy.sleep(0.2)
    head.set_pan(-0.5, speed=1.0)
    rospy.sleep(0.5)
    head.set_pan(0.0, speed=0.05)
    head.command_nod()

def main():
    """Demonstrate moving the head joints."""

    print("Initializing node...")
    rospy.init_node('test_head')
    print("Getting robot state...")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    
    rospy.on_shutdown(clean_shutdown)
    head_demo()
    

if __name__ == '__main__':
    main()