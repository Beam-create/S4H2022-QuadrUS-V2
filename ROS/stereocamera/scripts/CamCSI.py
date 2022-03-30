#!/usr/bin/env python3
import cv2
import threading
import numpy as np


class CSI_Camera:

    def __init__(self, sensor_id):
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False
        self.sensor_id = sensor_id
        self.gstreamer_pipeline_string = (
            "nvarguscamerasrc sensor-id=%d sensor-mode=3 ! "
            "video/x-raw(memory:NVMM), width=(int)1640, height=(int)1232, framerate=(fraction)30/1 ! "
            "nvvidconv flip-method=2 ! "
            "video/x-raw, width=(int)960, height=(int)720, format=(string)NV12 ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"%self.sensor_id)

    def open(self):
        try:
            self.video_capture = cv2.VideoCapture(
                self.gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )
            # Grab the first frame to start the video capturing
            self.grabbed, self.frame = self.video_capture.read()

        except RuntimeError:
            self.video_capture = None
            print("Erreur - Impossible d'ouvrir les cameras")
            print("Pipeline Gstreamer: " + self.gstreamer_pipeline_string)


    def start(self):
        if self.running:
            print("Le stream est en cours d'execution")
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        return self

    def stop(self):
        self.running = False
        # Kill the thread
        self.read_thread.join()
        self.read_thread = None

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Impossible d'acquerir l'image de la camera")
        # FIX ME - stop and cleanup thread
        # Something bad happened

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()


""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080
"""



def run_cameras():
    window_title = "Dual CSI Cameras"
    left_camera = CSI_Camera(1)
    left_camera.open()
    left_camera.start()

    right_camera = CSI_Camera(0)
    right_camera.open()
    right_camera.start()

    if left_camera.video_capture.isOpened() and right_camera.video_capture.isOpened():

        cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)

        try:
            while True:
                _, left_image = left_camera.read()
                _, right_image = right_camera.read()
                
                camera_images = np.hstack((left_image, right_image)) 
              
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, camera_images)
                else:
                    break

                keyCode = cv2.waitKey(10) & 0xFF
                if keyCode == 27:
                    break
        finally:

            left_camera.stop()
            left_camera.release()
            right_camera.stop()
            right_camera.release()
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to open both cameras")
        left_camera.stop()
        left_camera.release()
        right_camera.stop()
        right_camera.release()



if __name__ == "__main__":
    run_cameras()

