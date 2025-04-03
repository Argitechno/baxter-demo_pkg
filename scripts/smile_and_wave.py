#!/usr/bin/env python
# license removed for brevity
import struct
import rospy
import baxter_interface
import numpy as np
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

#Goal:
    #1 Display normal face & neutral position/praying mantis/untucked.
    #2 Move head to face window. (right)
    #3 Display smile / progress to smile while next step?
    #4 Move right arm to "hip"
    #5 Move left arm to wave position
    #6 Wave
    #7 Open and close gripper
    #7 Nod & Flash Lights
    #8 Back to neutral position/praying mantis/untucked

def get_pose(pos_list, quat_list):
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    return PoseStamped(
        header = hdr,
        pose = Pose(
            position=Point(pos_list[0], pos_list[1], pos_list[2]),
            orientation = Quaternion(quat_list[0], quat_list[1], quat_list[2], quat_list[3]),
        )
    )

def ik_get(limb, pose):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    ikreq.pose_stamp.append(pose)

    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print("SUCCESS - Valid Joint Solution Found")
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return 0


def main():
    print("Initializing node...")
    rospy.init_node('smile_and_wave')
    head = baxter_interface.Head()
    limb_left = baxter_interface.Limb('left')
    limb_right = baxter_interface.Limb('right')

    left_hip   = ik_get('left',  get_pose( [  0.1544,  0.4025,  0.0432 ], np.sqrt( [  0.4800,  0.0300, -0.4375,  0.0525 ] ) ) )
    right_wave = ik_get('right', get_pose( [ -0.2365, -0.9438,  1.0238 ], np.sqrt( [  0.0000,  0.0000,  0.5000,  0.5000 ] ) ) )
    print("Left Hip: ",  get_pose( [  0.1544,  0.4025,  0.0432 ], np.sqrt( [  0.4800,  0.0300, -0.4375,  0.0525 ] ) ) )
    print("Right Hip: ", get_pose( [ -0.2365, -0.9438,  1.0238 ], np.sqrt( [  0.0000,  0.0000,  0.5000,  0.5000 ] ) ) )
    if(left_hip == 0 or left_hip == 1 or right_wave == 0 or right_wave == 1): 
        return
    #1
    #2
    head.set_pan(-np.pi/2.4, speed=0.05)
    #3
    #4
    limb_left.move_to_joint_positions(left_hip)
    #5
    limb_right.move_to_joint_positions(right_wave)
    #6
    #7
    head.command_nod()
    #8
    rospy.sleep(3)
    head.set_pan(0, speed=0.5)

if __name__ == '__main__':
    main()