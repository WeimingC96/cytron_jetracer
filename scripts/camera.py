#!/usr/bin/env python3

import cv2
import numpy as np

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen

# Gstreamer pipeline settings
def gstreamer_pipeline(
    capture_width=600,
    capture_height=400,
    display_width=600,
    display_height=400,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc! \n"
        "video/x-raw(memory:NVMM), \n"
        "width=(int)%d, height=(int)%d, \n"
        "format=(string)NV12, framerate=(fraction)%d/1! \n"
        "nvvidconv flip-method=%d! \n"
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx! \n"
        "videoconvert! \n"
        "video/x-raw, format=(string)BGR ! appsink \n"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


#Capture video frame
def show_camera():
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print("flip method: ", gstreamer_pipeline(flip_method=0))
    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    #cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! videoconvert ! video/x-raw, format=BGR ! appsink", cv2.CAP_GSTREAMER)
    if cap.isOpened():
        window_handle = cv2.namedWindow("CSI Camera", cv2.WINDOW_AUTOSIZE)
        # Window
        while cv2.getWindowProperty("CSI Camera", 0) >= 0:
            ret_val, img = cap.read()
            cv2.imshow("CSI Camera", img)
            ret, buffer = cv2.imencode('.jpg', img)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result
            
            keyCode = cv2.waitKey(30) & 0xFF
            # Stop the program on the ESC key
            if keyCode == 27:
                break
        cap.release()
        cv2.destroyAllWindows()
    else:
        print("Unable to open camera")

if __name__ == "__main__":
    show_camera()