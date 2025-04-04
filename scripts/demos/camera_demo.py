#!/usr/bin/env python
# license removed for brevity
import argparse
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
from baxter_interface.src.baxter_interface.camera import CameraController

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

class CameraLink():
    def __init__(self, camera_open, res, camera_close):
        """Establishes a new camera link to camera_open, closing camera_close if both other cameras are open."""
        self._cv_image = None
        print("Getting camera list.")
        cameras = list_cameras()
        camera_count = 0
        for key, value in cameras.items():
            if value:
                camera_count += 1
        
        if(not cameras.get(camera_open, False) and  camera_count > 1):
            print("Closing %s." % (camera_close))
            close_cam(camera_close)

        print("Opening %s." % (camera_open))
        open_cam(camera_open, res)

        print("Opening bridge.")
        self._bridge = cv_bridge.CvBridge()

        print("Opening subscriber to image.")
        rospy.Subscriber('/cameras/head_camera/image', Image, self._image_callback)
        
    def _image_callback(self, ros_img):
        self._cv_image = self._bridge.imgmsg_to_cv2(ros_img, desired_encoding = "passthrough")  

    def get_image(self):
        return self._cv_image


def display_link(camera_link):
        """Display the image from camera_link on the running machine."""
        cv_image = camera_link.get_image()
        if(cv_image is not None):
            cv2.imshow('Image', cv_image)

def head_stream():
    """Continously stream the head camera to the host screen until the node is shutdown."""

    head_cam_link = CameraLink('head_camera', (640, 400), 'left_hand_camera')
    rate = rospy.Rate(60)
    print("Hit ^C in terminal or q in window to exit stream.")
    while not rospy.is_shutdown():
        display_link(head_cam_link)
        if cv2.waitKey(1) & 0xFF == ord('q'):
          break
        rate.sleep()


def main():
    """Demonstrate access to camera output."""
    
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(   formatter_class = arg_fmt,
                                        description = main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing Node...")
    rospy.init_node('camera_demo', anonymous=True)
    print("Getting robot state...")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    
    def clean_shutdown():
        print("\nExiting example...")
        cv2.destroyAllWindows()
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    
    rospy.on_shutdown(clean_shutdown)
    head_stream()
  
if __name__ == '__main__':
    main()
