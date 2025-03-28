#!/usr/bin/env python
# license removed for brevity
import rospy
import cv2
import cv_bridge
import os.path


from sensor_msgs.msg import Image

def main():
    print("Checking Video")
    rickroll = os.path.dirname(os.path.abspath(__file__))
    rickroll = rickroll + '/../../assets/rickroll.mp4'
    if not os.path.isfile(rickroll):
        print("No file exists at: ")
        print(rickroll)
        return
    
    print("Initializing Node")
    rospy.init_node('screen_demo', anonymous=True)

    print("Opening Publisher")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size = 1)

    print("Opening Video")
    video = cv2.VideoCapture(rickroll)

    print("Link Start!")
    fps = video.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(fps)
    while not rospy.is_shutdown():
        _, frame = video.read()
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)
        k = cv2.waitKey(1) & 0xFF
        rate.sleep()
    
    video.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()





