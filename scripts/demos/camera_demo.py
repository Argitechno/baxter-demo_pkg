#!/usr/bin/env python
# license removed for brevity
import argparse
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_interface.camera import CameraController

import rosgraph
import socket
import cv2
import cv_bridge

from sensor_msgs.msg import Image
from baxter_core_msgs.srv import ListCameras


def open_cam(camera, res):
    """Open the given camera at the set resolution if valid."""

    if not any((res[0] == r[0] and res[1] == r[1]) for r in CameraController.MODES):
        rospy.logerr("Invalid resolution provided.")
    cam = CameraController(camera) 
    cam.resolution = res 
    cam.open()

def close_cam(camera):
    """Close the given camera."""

    cam = CameraController(camera)
    cam.close()

def list_cameras():
    """Get dictionary with camera names and whether they are active or not."""

    ls = rospy.ServiceProxy('cameras/list', ListCameras)
    rospy.wait_for_service('cameras/list', timeout = 10)
    resp = ls()
    if len(resp.cameras):
        master = rosgraph.Master('/rostopic')
        resp.cameras
        cam_topics = dict([(cam, "/cameras/%s/image" % cam) for cam in resp.cameras])
        open_cams = dict([(cam, False) for cam in resp.cameras])
        try:
            topics = master.getPublishedTopics('')
            for topic in topics:
                for cam in resp.cameras:
                    if topic[0] == cam_topics[cam]:
                            open_cams[cam] = True
        except socket.error:
            raise ROSTopicIOException("Cannot communicate with master.")
    return open_cams



def main():
    """Demonstrate access to camera output."""
    
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(   formatter_class = arg_fmt,
                                        description = main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing Node")
    rospy.init_node('camera_demo', anonymous=True)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    cv_image = 0
    def link_camera():
        """Start a link to the head camera, closing the left hand camera if needed, and displaying the image on the running machine."""
        nonlocal cv_image

        print("Getting camera list")
        cameras = list_cameras()

        if(not cameras.get('head_camera', False) and cameras.get('left_hand_camera', False) and cameras.get('right_hand_camera', False)):
            print("Closing left hand camera")
            close_cam('left_hand_camera')
        print("Opening head camera")
        open_cam('head_camera', (640, 400))

        print("Opening bridge")
        bridge = cv_bridge.CvBridge()

        
        def image_callback(ros_img):
            nonlocal cv_image 
            cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding = "passthrough")

        print("Opening subscriber to image.")
        rospy.Subscriber('/cameras/head_camera/image', Image, image_callback)

    def clean_shutdown():
        print("\nExiting example...")
        cv2.destroyAllWindows()
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    
    rospy.on_shutdown(clean_shutdown)
    link_camera()

    rate = rospy.rate(60)
    while not rospy.is_shutdown():
        if(cv_image != 0):
            cv2.imshow('Image', cv_image)
            cv2.waitKey(1)
        rate.sleep()
    rospy.spin()
  
if __name__ == '__main__':
    main()
