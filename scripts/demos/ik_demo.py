#!/usr/bin/env python
# license removed for brevity
import struct
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header

import baxter_interface

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
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
    print("Initializing Node.")
    rospy.init_node("ik_demo")
    limb_left = baxter_interface.Limb('left')
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')

    quat_tf = [1, 0, 0, 0]
    pose1 =  PoseStamped(
        header = hdr,
        pose = Pose(
            position=Point(
                x =  0.8,
                y = -0.3,
                z =  0.3,
            ),
            orientation = Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3]),
        )
    )
    print('IK Joint Solution: ')
    print(ik_get('left', pose1))
    if(pose1 != 0 and pose1 != 1):
        limb_left.move_to_joint_positions(pose1)
if __name__ == '__main__':
    main()