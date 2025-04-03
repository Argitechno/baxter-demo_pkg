#!/usr/bin/env python
# license removed for brevity
import rospy
import baxter_interface
import numpy as np
#Goal:
    #1 Display normal face.
    #2 Move head to face window. (right)
    #3 Display smile / progress to smile while next step?
    #4 Move right arm to "hip"
    #5 Move left arm to wave position
    #6 Wave
    #7 Open and close gripper
    #7 Nod & Flash Lights
    #8 Back to neutral position/praying mantis/untucked
def main():
    print("Initializing node...")
    rospy.init_node('smile_and_wave')
    head = baxter_interface.Head()

    #1
    #2
    head.set_pan(-np.PI/2, speed=0.05)
    #3
    #4
    #5
    #6
    #7
    head.command_nod()
    #8
    head.set_pan(0, speed=0.5)

if __name__ == '__main__':
    main()