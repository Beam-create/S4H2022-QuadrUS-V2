#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import threading
import numpy as np

bridge = CvBridge()

""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def cam_pub():
    window_title = "CSI Camera"
    
    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
    pub = rospy.Publisher('/CAM0_raw', Image, queue_size=1)
    rospy.init_node('image', anonymous=False)
    rate = rospy.Rate(10)
    if video_capture.isOpened():
        try:    
            while True:
                ret_val, frame = video_capture.read()
                msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                
                pub.publish(msg)
                if cv2.waitKey(1) & 0xFF== ord('q'):
                    break
                if rospy.is_shutdown():
                    break
        finally:
            video_capture.release()
    else:
        print("Error: Unable to open camera")


if __name__ == "__main__":
    try:
        cam_pub()
    except rospy.ROSInterruptException:
        pass
