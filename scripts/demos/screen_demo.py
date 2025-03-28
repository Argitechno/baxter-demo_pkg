#!/usr/bin/env python
# license removed for brevity
import rospy
import cv2
import cv_bridge
import os.path


from sensor_msgs.msg import Image

def resize(img):
    height, width, layers = img.shape
    scale = min(600/height, 1024/width)
    print("(%f, %f) to (%f, %f) Scale: %f" % (width, height, width * scale, height * scale, scale))
    return cv2.resize(img, (scale * width, scale * height))
    

def main():
    print("Checking Video")
    base_dir = os.path.dirname(os.path.abspath(__file__))
    rickroll = base_dir + '/../../assets/rickroll.mp4'
    if not os.path.isfile(rickroll):
        print("No file exists at: ")
        print(rickroll)
        return
    
    print("Initializing Node")
    rospy.init_node('screen_demo', anonymous=True)

    print("Opening Publisher")
    pubVid = rospy.Publisher('/robot/xdisplay', Image, queue_size = 1)
    pubImg = rospy.Publisher('/robot/xdisplay', Image, latch = True, queue_size = 1)
    print("Opening Video")
    video = cv2.VideoCapture(rickroll)
    
    print("Link Start!")
    fps = video.get(cv2.CAP_PROP_FPS)
    rate = rospy.Rate(fps)
    targetExit = rospy.Time.now() + rospy.Duration(20)
    while not rospy.is_shutdown() and rospy.Time.now() < targetExit:
        _, frame = video.read()
        frame = resize(frame)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
        pubVid.publish(msg)
        rate.sleep()
    
    evil_baxter = base_dir + '/../../assets/evil_baxter.jpeg'
    if not os.path.isfile(evil_baxter):
        print("No file exists at: ")
        print(evil_baxter)
        video.release()
        cv2.destroyAllWindows()
    img = cv2.imread(evil_baxter)
    img = resize(img)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pubImg.publish(msg)
    rospy.sleep(1)

    video.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()





