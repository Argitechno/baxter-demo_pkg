#!/usr/bin/env python
# license removed for brevity
import rospy
import cv2
import cv_bridge
from sensor_msgs.msg import Image


def main():
    rospy.init_node('screen_demo', anonymous=True)
    pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size = 1)
    video = cv2.VideoCapture('../../assets/rickroll.mp4')

    rate = rospy.Rate(60)
    while True:
        _, frame = video.read()
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break   
        rate.sleep()
    
    video.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()





