#!/usr/bin/env python3

from jetcam.csi_camera import CSICamera
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge


bridge = CvBridge()
camera = CSICamera(width=224, height=224)
image = camera.read()
image_pub = rospy.Publisher("/motion_image", Image, queue_size=10)

#We can then capture a frame from the camera like this

img = bridge.cv2_to_imgmsg(image, "bgr8")
image_pub.publish(img)
