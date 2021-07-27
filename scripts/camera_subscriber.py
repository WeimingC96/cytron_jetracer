#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

class Camera:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic", CompressedImage, self.image_callback, queue_size = 100, buff_size=5000)

    def image_callback(self, msg):
        #print ("Processing frame | Delay:%6.3f" % (rospy.Time.now() - msg.header.stamp).to_sec())
        try:
            print("image converted")
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        """ (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50,50), 10, 255) """
        
        cv2.imshow("image", cv_image)
        cv2.waitKey(3)

def main():
    cam = Camera()
    rospy.init_node('image_sub', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
            print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
        