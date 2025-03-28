#!/usr/bin/env python
# license removed for brevity
import rospy
import baxter_interface


def main():
    print("Initializing node...")
    rospy.init_node('test_head')
    print("Begin scan")
    head = baxter_interface.Head()
    head.set_pan(0.5, speed=0.05)
    rospy.sleep(0.2)
    head.set_pan(-0.5, speed=1.0)
    rospy.sleep(0.5)
    head.set_pan(0.0, speed=0.05)
    head.command_nod()
    

if __name__ == '__main__':
    main()