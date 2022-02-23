import cv2
import numpy as np
from CamCSI import *



def acquisition():
    left_camera = CSI_Camera(1)
    left_camera.open()
    left_camera.start()

    right_camera = CSI_Camera(0)
    right_camera.open()
    right_camera.start()

    if left_camera.video_capture.isOpened() and right_camera.video_capture.isOpened():

        cv2.namedWindow('Camera GAUCHE', cv2.WINDOW_AUTOSIZE)
        cv2.namedWindow('Camera DROITE', cv2.WINDOW_AUTOSIZE)
        counter = 20
        try:
            while True:
                _, left_image = left_camera.read()
                _, right_image = right_camera.read()
              
                if cv2.getWindowProperty('Camera GAUCHE', cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow('Camera GAUCHE', left_image)
                    cv2.imshow('Camera DROITE', right_image)
                else:
                    break
                
                keyCode = cv2.waitKey(10) & 0xFF
                if keyCode == ord(' '):
                    cv2.imwrite("Images/Gauche/imgG"+str(counter)+".png", left_image)
                    cv2.imwrite("Images/Droite/imgD"+str(counter)+".png", right_image)
                    print("Enregistrement...")
                    counter += 1

                if keyCode == 27:
                    break
        finally:
            left_camera.stop()
            left_camera.release()
            right_camera.stop()
            right_camera.release()
        cv2.destroyAllWindows()
    else:
        print("Erreur : Impossible d'ouvrir les cameras")
        left_camera.stop()
        left_camera.release()
        right_camera.stop()
        right_camera.release()



if __name__ == "__main__":
    acquisition()

