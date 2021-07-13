#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def process_image(msg):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow("image", img)
    cv2.waitKey(50)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node('image_sub')
        rospy.loginfo('image_sub node started')
        rospy.Subscriber("/gi/simulation/left/image_raw", Image, process_image)

        rospy.spin()