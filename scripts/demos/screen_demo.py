#!/usr/bin/env python
# license removed for brevity
import rospy
import cv2
import cv_bridge
import os.path


from sensor_msgs.msg import Image

def main():
    rickroll = '../../assets/rickroll.mp4'
    if not os.path.isfile(rickroll):
        print("No file exists at: ")
        print(rickroll)
        return
    rospy.init_node('screen_demo', anonymous=True)
    pub = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size = 1)
    video = cv2.VideoCapture(os.path(rickroll))

    fps = video.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(fps)
    while True:
        _, frame = video.read()
        print(frame)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
        pub.publish(msg)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break   
        rate.sleep()
    
    video.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()





