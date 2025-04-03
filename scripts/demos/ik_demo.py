#!/usr/bin/env python
# license removed for brevity
import struct
import rospy
import numpy as np
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
    print("Initializing Node.")
    rospy.init_node("ik_demo")
    limb_left = baxter_interface.Limb('left')
    pos = [0.6, 0.600, 0.35]

    #The default rotation (0, 0, 0, 1) faces the wrist up.
    quat_tf = [0, 0, 0, 1]
    pose = ik_get('left', get_pose(pos, quat_tf))
    if(pose != 0 and pose != 1):
        limb_left.move_to_joint_positions(pose)
    
    #This quaternion defines a rotation around the X axis and the Y axis.
    #This is a full rotation as w is 0 (w = 2*arccos(theta))
    #And so: the resulting is that since we are rotating 180 around an axis that has no Z (vertical) component, is that the arm will face down.
    #However: the rotation of the wrist: since when in base position, the wrist is unrotated.
    #Now, the left side of the gripper would be in the positive x (away from the robot) direction, and the right side would be in negative y.
    #So rotations affect the gripper orientation as well, not just the arm.
    #And the axis isn't exactly 135 degrees from positive x, so the grippers will actually be (likely unoticably) rotated a bit counter clockwise if looking down, 
    #rather than perfectly 90 degrees
    quat_tf = [0.6614378278, 0.75, 0, 0]

    #Visit all 8 corners of a small cube centered around the current point
    s=0.1
    for i in range(-1, 2, 2):
        for j in range(-1, 2, 2):
            for k in range(-1, 2, 2):
                pose = ik_get('left', get_pose(np.add(pos, [ s*i, s*j, s*k ]), quat_tf))
                if(pose != 0 and pose != 1):
                    limb_left.move_to_joint_positions(pose)
    
    #Return to original point, but change the orientation back to face down and normal gripper orientation, upside down
    quat_tf = [1, 0, 0, 0]
    pose = ik_get('left', get_pose(pos, quat_tf))
    if(pose != 0 and pose != 1):
        limb_left.move_to_joint_positions(pose)

if __name__ == '__main__':
    main()