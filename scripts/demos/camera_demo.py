#!/usr/bin/env python
# license removed for brevity
import rospy
import rosgraph
import socket
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from baxter_core_msgs.srv import ListCameras
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

def list_cameras():
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
    print("Initializing Node")
    rospy.init_node('camera_demo', anonymous=True)

    print("Getting camera list")
    cameras = list_cameras()

    if(not cameras.get('head_camera', False) and cameras.get('left_hand_camera', False) ):
        print("Closing left hand camera")
        close_cam('left_hand_camera')
    print("Opening head camera")
    open_cam('head_camera', (1280, 800))

    print("Opening bridge")
    bridge = cv_bridge.CvBridge()
    def image_callback(ros_img):
        cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding = "passthrough")
        cv2.imshow('Image', cv_image)
        cv2.waitKey(1)

    print("Opening subscriber to image.")
    rospy.Subscriber('/cameras/torso_camera/image', Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main()
