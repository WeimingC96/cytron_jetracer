#!/usr/bin/env python

import cv2
import numpy as np
import rospy
import gi
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject


# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen

class Camera:
    
    def __init__(self):
        #self.image_pub = rospy.Publisher("image_topic", Image, queue_size=1)
        self.bridge = CvBridge()
        self.display = False
        self.capture_width=600
        self.capture_height=400
        self.display_width=600
        self.display_height=400
        self.framerate=30
        self.flip_method=0
    
    def gst_to_opencv(self, gst_buffer):
        return np.ndarray((480,640,3), buffer=gst_buffer.extract_dup(0, gst_buffer.get_size()), dtype=np.uint8)

    def gCamera(self):
        image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
        rospy.init_node('camera_image', anonymous=True)
        Gst.init(None)
        pipe = Gst.parse_launch("""v4l2src device=/dev/video0 ! video/x-raw, width=640, height=480,format=(string)BGR ! appsink sync=false max-buffers=2 drop=true name=sink emit-signals=true""")
        sink = pipe.get_by_name('sink')
        pipe.set_state(Gst.State.PLAYING)
        while not rospy.is_shutdown():
            sample = sink.emit('pull-sample')
            if (sample != None):
                img = self.gst_to_opencv(sample.get_buffer())
                try:
                    image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
                except CvBridgeError as e:
                    print(e)
            else:
                print("sample is none!")


    # Gstreamer pipeline settings
    def gstreamer_pipeline(self):
        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                self.capture_width,
                self.capture_height,
                self.framerate,
                self.flip_method,
                self.display_width,
                self.display_height
            )
        )

    #Capture video frame
    def show_camera(self):
        # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
        cap = cv2.VideoCapture(self.gstreamer_pipeline(), cv2.CAP_GSTREAMER)
        #cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! videoconvert ! video/x-raw, format=BGR ! appsink", cv2.CAP_GSTREAMER)
        if not self.display:
            if cap.isOpened():
                image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
                rospy.init_node('camera_image', anonymous=True)
                print("camera opened")
                while not rospy.is_shutdown():
                    try:
                        ret_val, img = cap.read()
                        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
                    except CvBridgeError as e:
                        print(e)
            else:
                print("Unable to open camera")

        else:
            if cap.isOpened():
                print("camera opened")
                window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
                # Window
                while cv2.getWindowProperty("CSI Camera", 0) >= 0:
                    ret_val, img = cap.read()
                    cv2.imshow("CSI Camera", img)
                    keyCode = cv2.waitKey(30) & 0xFF
                    # Stop the program on the ESC key
                    if keyCode == 27:
                        break
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
                cap.release()
                cv2.destroyAllWindows()
            else:
                print("Unable to open camera")

def main():
    cam = Camera()
    cam.show_camera()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("Shutting down")
        pass
    
        

        
        