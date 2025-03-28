#!/usr/bin/env python
# license removed for brevity
import rospy
import os
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from baxter_interface.camera import CameraController

def open_cam(camera, res):
    if not any((res[0] == r[0] and res[1] == r[1]) for r in CameraController.MODES):
        rospy.logerr("Invalid resolution provided.")
    cam = CameraController(camera) 
    cam.resolution = res 
    cam.open()

def close_cam(camera):
    cam = CameraController(camera)
    cam.close()

def main():
    rospy.init_node('camera_demo', anonymous=True)
    close_cam('left_hand_camera')
    open_cam('torso_camera')
    bridge = cv_bridge.CvBridge()
    def image_callback(ros_img):
        cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding = "passthrough")
        cv2.imshow('Image', cv_image)
        cv2.waitKey(1)
    rospy.Subscriber('/cameras/torso_camera/image', Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()
