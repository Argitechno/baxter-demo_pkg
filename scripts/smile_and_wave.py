#!/usr/bin/env python
# license removed for brevity
import struct
import rospy
import baxter_interface
import numpy as np
import cv2
import cv_bridge
import os.path
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from std_msgs.msg import Header
from sensor_msgs.msg import Image
import baxter_interface.digital_io as DIO

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
    #8 Nod & Flash Lights
    #9 Back to neutral position/praying mantis/untucked

def resize(img):
    height, width, layers = img.shape
    scale = min(600.0/height, 1024.0/width)
    print("(%f, %f) to (%f, %f) Scale: %f" % (width, height, width * scale, height * scale, scale))
    return cv2.resize(img, (int(scale * width), int(scale * height)))

def border(img):
    height, width, layers = img.shape
    verPad = (600 - height)/2
    horPad = (1024 - width)/2
    return cv2.copyMakeBorder(img, verPad, height-verPad, horPad, width-horPad, cv2.BORDER_CONSTANT, value = (0,0,0))

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

    print("Opening Publisher")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size = 1)

    print("Initializing interfaces...")
    head = baxter_interface.Head()
    limb_left = baxter_interface.Limb('left')
    limb_right = baxter_interface.Limb('right')
    gripper = baxter_interface.Gripper('right')

    base_dir = os.path.dirname(os.path.abspath(__file__))

    #0 Calculate positions.
    print("Getting IK Positions...")
    left_hip    = ik_get('left',  get_pose( [  0.2238,  0.4261,  0.0395 ], np.sqrt( [ 0.5246, 0.0234, 0.4006, 0.0514 ] ) * [  1,  1, -1,  1] ) )
    right_wave0 = ik_get('right', get_pose( [ -0.2365, -0.9438,  1.0238 ], np.sqrt( [ 0.0000, 0.0000, 0.5000, 0.5000 ] ) * [  1,  1,  1,  1] ) )
    wave_move = {'right_s0': 0.1, 'right_s1': 0.2, 'right_w0': -0.7, 'right_w1': 0.2, 'right_w2': 0, 'right_e0': -0.3, 'right_e1': 0.4}
    right_wave1 = {k: right_wave0.get(k, 0) + wave_move.get(k, 0) for k in right_wave0.keys()}
    if(
        left_hip    == 0 or left_hip    == 1 or 
        right_wave0 == 0 or right_wave0 == 1
    ):
        return
    
    #1 Display normal face & neutral position/praying mantis/untucked.
    print("Initialize baxter...")

    #Face
    neutral_baxter = base_dir + '/../assets/baxter_face_neutral.png'
    if not os.path.isfile(neutral_baxter):
        print("No file exists at: ")
        print(neutral_baxter)
        return
    img = cv2.imread(neutral_baxter)
    img = resize(img)
    img = border(img)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(1)


    #Head Position
    head.set_pan(0, speed=0.5)
    #Arm Positions
    left_pos =  { 'left_e0'  :  -2*np.pi/5, 'left_e1'  :  3*np.pi/5, 'left_s0'  : 0, 'left_s1'  :  -3*np.pi/10, 'left_w0'  :   np.pi/5, 'left_w1'  :  3*np.pi/10, 'left_w2'  :  -np.pi/5}
    right_pos = { 'right_e0' :   2*np.pi/5, 'right_e1' :  3*np.pi/5, 'right_s0' : 0, 'right_s1' :  -3*np.pi/10, 'right_w0' :  -np.pi/5, 'right_w1' :  3*np.pi/10, 'right_w2' :   np.pi/5}
    limb_left.move_to_joint_positions(left_pos)
    limb_right.move_to_joint_positions(right_pos)
    rospy.sleep(1)
    
    #2 Move head to face window. (right)
    print("Face window...")
    head.set_pan(-np.pi/2.4, speed=0.05)
    
    #3 Display smile / progress to smile while next step?
    print("Smile...")
    happy_baxter = base_dir + '/../assets/baxter_face_happy.png'
    if not os.path.isfile(happy_baxter):
        print("No file exists at: ")
        print(happy_baxter)
        return
    img = cv2.imread(happy_baxter)
    img = resize(img)
    img = border(img)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(1)

    #4 Move right arm to "hip"
    print("Right arm to hip...")
    limb_left.move_to_joint_positions(left_hip)
    
    #5 Move left arm to wave position
    print("Left arm to wave...")
    limb_right.move_to_joint_positions(right_wave0)

    #6 Wave * 2
    print("Wave * 2...")
    limb_right.move_to_joint_positions(right_wave1)
    limb_right.move_to_joint_positions(right_wave0)
    limb_right.move_to_joint_positions(right_wave1)
    limb_right.move_to_joint_positions(right_wave0)
    
    #7 Open and close gripper
    print("Open & Close Gripper...")
    gripper.calibrate()
    gripper.open()
    gripper.close()
    gripper.open()

    #8 Nod & Flash Lights
    print("Nod & Flash Lights...")
    head.command_nod()
    leftLightInner = DIO.DigitalIO('left_inner_light')
    leftLightOuter = DIO.DigitalIO('left_outer_light')
    rightLightInner = DIO.DigitalIO('right_inner_light')
    righLightOuter = DIO.DigitalIO('right_outer_light')
    torsoLeftLightInner = DIO.DigitalIO('torso_left_inner_light')
    torsoLeftLightOuter = DIO.DigitalIO('torso_left_outer_light')
    torsoRightLightInner = DIO.DigitalIO('torso_right_inner_light')
    torsoRightLightOuter = DIO.DigitalIO('torso_right_outer_light')

    def switchState(digitalComponent):
	    digitalComponent.state = not digitalComponent.state

    def switchLights():
	    switchState(leftLightInner)
	    switchState(leftLightOuter)
	    switchState(rightLightInner)
	    switchState(righLightOuter)
	    switchState(torsoLeftLightInner)
	    switchState(torsoLeftLightOuter)
	    switchState(torsoRightLightInner)
	    switchState(torsoRightLightOuter)
	

    for i in range(0, 10):
		switchLights()
		print("Switched the lights!", i)
		rospy.sleep(0.2)

    #9 Back to neutral position/praying mantis/untucked
    img = cv2.imread(neutral_baxter)
    img = resize(img)
    img = border(img)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub.publish(msg)
    rospy.sleep(1)
    limb_left.move_to_joint_positions(left_pos)
    limb_right.move_to_joint_positions(right_pos)
    rospy.sleep(3)
    head.set_pan(0, speed=0.5)

if __name__ == '__main__':
    main()