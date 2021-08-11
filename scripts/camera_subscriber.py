#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

class Camera:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic", CompressedImage, self.image_callback, queue_size = 10)
        self.raw_image_pub = rospy.Publisher("raw_image_topic", Image, queue_size = 10)

    def image_callback(self, msg):
        try:
            print("image converted")
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            ros_img = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.raw_image_pub.publish(ros_img)
        except CvBridgeError as e:
            print(e)
        
        """ cv2.imshow("image", cv_image)
        cv2.waitKey(3) """

def main():
    rospy.init_node('image_sub', anonymous=True)
    cam = Camera()
    try:
        rospy.spin()
    except KeyboardInterrupt:
            print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
        