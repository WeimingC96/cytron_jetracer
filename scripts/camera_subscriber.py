#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

bridge = CvBridge()

def process_image(msg):
    try:
        print("image converted")
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
    except CvBridgeError as e:
        print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60:
        cv2.circle(cv_image, (50,50), 10, 255)
    
    cv2.imshow("image", cv_image)
    cv2.waitKey(50)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            rospy.init_node('image_sub', anonymous=True)
            rospy.loginfo('image_sub node started')
            rospy.Subscriber("image_topic", Image, process_image)

            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")
        cv2.destroyAllWindows()