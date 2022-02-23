import sys
import cv2
import numpy as np
import time
import imutils
from matplotlib import pyplot as plt
from CamCSI import *

import triangulation as tri


import HSV_filter as hsv
import shape_recognition as shape

import mediapipe as mp
import time
def detection():
    mp_facedetector = mp.solutions.face_detection
    mp_draw = mp.solutions.drawing_utils



    # Donnees pour la stereoscopie
    frame_rate = 120    #FPS
    B = 6               #Distance entre les cameras [cm]
    f = 2.6             #Distance focale [mm]
    alpha = 73          #Angle de la lentille sur la plan horizontal[degrees]


    cv_file = cv2.FileStorage()
    cv_file.open("stereoMap.xml", cv2.FileStorage_READ)

    stereoMapL_x = cv_file.getNode('stereoMapL_x').mat()
    stereoMapL_y = cv_file.getNode('stereoMapL_y').mat()
    stereoMapR_x = cv_file.getNode('stereoMapR_x').mat()
    stereoMapR_y = cv_file.getNode('stereoMapR_y').mat()
    cv_file.release()
    
   
    cap_left = CSI_Camera(1)
    cap_left.open()
    cap_left.start()

    cap_right = CSI_Camera(0)
    cap_right.open()
    cap_right.start()
    if cap_left.video_capture.isOpened() and cap_right.video_capture.isOpened():
        cv2.namedWindow('Camera de droite', cv2.WINDOW_KEEPRATIO)
        # Programme principal - detection des visages et des balles avec estimation de la profondeur 
        with mp_facedetector.FaceDetection(min_detection_confidence=0.7) as face_detection:

            try:
                while True:
                    

                    succes_right, frame_right = cap_right.read()
                    succes_left, frame_left = cap_left.read()

                ################## CALIBRATION #########################################################

                    
                    frame_left= cv2.remap(frame_left, stereoMapL_x, stereoMapL_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)
                    frame_right= cv2.remap(frame_right, stereoMapR_x, stereoMapR_y, cv2.INTER_LANCZOS4, cv2.BORDER_CONSTANT, 0)

                ########################################################################################

                    
                    if not succes_right or not succes_left:                    
                        break

                    else:

                        start = time.time()
                        
                        # Filtre HSV:
                        mask_right = hsv.add_HSV_filter(frame_right, 1)
                        mask_left = hsv.add_HSV_filter(frame_left, 0)

                        # Resultat du masque HSV applique a l'image
                        res_right = cv2.bitwise_and(frame_right, frame_right, mask=mask_right)
                        res_left = cv2.bitwise_and(frame_left, frame_left, mask=mask_left) 

                        # Reconnaissance des formes:
                        circles_right = shape.find_circles(frame_right, mask_right)
                        circles_left  = shape.find_circles(frame_left, mask_left)

                        if np.all(circles_right) == None or np.all(circles_left) == None:
                            cv2.putText(frame_right, "Balle perdue", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),1)
                           

                        else:
                            # Calcul de profondeur. Vecteurs contenant plusieurs profondeurs si multiples objets.
                            
                            depth = tri.find_depth(circles_right, circles_left, frame_right, frame_left, B, f, alpha)

                            cv2.putText(frame_right, "Balle - Distance [cm]: " + str(round(depth,3)), (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255),1)
                            

                        # Conversion de BGR a RGB
                        frame_right = cv2.cvtColor(frame_right, cv2.COLOR_BGR2RGB)
                        frame_left = cv2.cvtColor(frame_left, cv2.COLOR_BGR2RGB)

                        # Detection des visages 
                        face_results_right = face_detection.process(frame_right)
                        face_results_left = face_detection.process(frame_left)

                        # Converion RGB a BGR
                        frame_right = cv2.cvtColor(frame_right, cv2.COLOR_RGB2BGR)
                        frame_left = cv2.cvtColor(frame_left, cv2.COLOR_RGB2BGR)


                        ################## Calcul de la profondeur - Detection des visages #########################################################

                        center_right = 0
                        center_left = 0

                        if face_results_right.detections:
                            for id, detection in enumerate(face_results_right.detections):
                                mp_draw.draw_detection(frame_right, detection)

                                bBox = detection.location_data.relative_bounding_box

                                h, w, c = frame_right.shape

                                boundBox = int(bBox.xmin * w), int(bBox.ymin * h), int(bBox.width * w), int(bBox.height * h)

                                center_point_right = (boundBox[0] + boundBox[2] / 2, boundBox[1] + boundBox[3] / 2)

                                cv2.putText(frame_right, f'{int(detection.score[0]*100)}%', (boundBox[0], boundBox[1] - 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1)


                        if face_results_left.detections:
                            for id, detection in enumerate(face_results_left.detections):
                                bBox = detection.location_data.relative_bounding_box

                                h, w, c = frame_left.shape

                                boundBox = int(bBox.xmin * w), int(bBox.ymin * h), int(bBox.width * w), int(bBox.height * h)

                                center_point_left = (boundBox[0] + boundBox[2] / 2, boundBox[1] + boundBox[3] / 2)




                       
                        if not face_results_right.detections or not face_results_left.detections:
                            cv2.putText(frame_right, "Visage perdu", (20,70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),1)
                           
                        else:
                            
                            depth = tri.find_depth(center_point_right, center_point_left, frame_right, frame_left, B, f, alpha)

                            cv2.putText(frame_right, "Visage - Distance [cm]: " + str(round(depth,1)), (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0),1)
                            



                        end = time.time()
                        totalTime = end - start

                        fps = 1 / totalTime
                        print("FPS: ", fps)

                        # cv2.putText(frame_right, f'FPS: {int(fps)}', (20,450), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,255,0), 2)
                        

                        
                        cv2.imshow("Camera de droite", frame_right) 
                        # cv2.imshow("Camera de gauche", frame_left)


                        # Appuyer sur la touche 'q' pour fermer le programme
                        keyCode = cv2.waitKey(1) & 0xFF
                        if keyCode == 27:
                            break
            finally:
                cap_left.stop()
                cap_left.release()
                cap_right.stop()
                cap_right.release()
            cv2.destroyAllWindows()
    else:
        cap_left.stop()
        cap_left.release()
        cap_right.stop()
        cap_right.release()

if __name__ == "__main__":
    detection()
