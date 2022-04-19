#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
import time
import imutils
from matplotlib import pyplot as plt

sys.path.append('/home/jetson/catkin_ws/src/stereocamera/scripts')
from CamCSI import *
import triangulation2 as tri
import HSV_filter as hsv
import shape_recognition as shape



bridge = CvBridge()
def ball_stereo_detection_pub():


    # Donnees pour la stereoscopie
    frame_rate = 120    #FPS
    B = 6            #Distance entre les cameras [cm]
    f = 2.6             #Distance focale [mm]
    alpha = 73          #Angle de la lentille sur la plan horizontal[degrees]

    # pub_left = rospy.Publisher('/left_cam', Image, queue_size=1)
    rospy.init_node('camera', anonymous=False, )
    pub_right = rospy.Publisher('/camera/right_cam', Image, queue_size=1)
    pub_vector = rospy.Publisher('/camera/Ball_pos', Vector3, queue_size=10)
    
    rate = rospy.Rate(100)

    cap_left = CSI_Camera(0)
    cap_left.open()
    cap_left.start()

    cap_right = CSI_Camera(1)
    cap_right.open()
    cap_right.start()
    ball_position = Vector3()
    if cap_left.video_capture.isOpened() and cap_right.video_capture.isOpened():
        # Programme principal - detection des visages et des balles avec estimation de la profondeur 

        try:
            while True:
                

                succes_right, frame_right = cap_right.read()
                succes_left, frame_left = cap_left.read()
                # msg_right = bridge.cv2_to_imgmsg(cv2.resize(frame_right, (240,180), interpolation=cv2.INTER_NEAREST), "bgr8")
                # pub_right.publish(msg_right)

                if not succes_right or not succes_left:                    
                    break

                else:

                    # start = time.time()
                    
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
                        pass
                    #    cv2.putText(frame_right, "Balle perdue", (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),1)
                        

                    else:
                        # Calcul de profondeur. Vecteurs contenant plusieurs profondeurs si multiples objets.
                        
                        depth, angle, X_position, Z_position = tri.find_depth_and_angle(circles_right, circles_left, frame_right, frame_left, B, f, alpha)
                        
                        ball_position.x = Z_position
                        ball_position.y = 3.7
                        ball_position.z = X_position
                        pub_vector.publish(ball_position)
                        
                    
                    msg_right = bridge.cv2_to_imgmsg(cv2.resize(frame_right, (240,130), interpolation=cv2.INTER_NEAREST), "bgr8")
                    # pub_right.publish(msg_right)
                    
                    # msg_left = bridge.cv2_to_imgmsg(cv2.resize(frame_left, (720,540), interpolation=cv2.INTER_NEAREST), "bgr8")
                    #frame_right_comp = cv2.imencode('.jpg', frame_right)
                    #frame_left_comp = cv2.imencode('.jpg', frame_left) 
                    # msg_right = bridge.cv2_to_compressed_imgmsg(frame_right, dst_format='jpeg')
                    #msg_left = bridge.cv2_to_compressed_imgmsg(frame_left, dst_format='jpg')
                    # pub_left.publish(msg_left)
                    # end = time.time()
                    # totalTime = end - start

                    # fps = 1 / totalTime
                    # print("FPS: ", fps)
                    # keyCode = cv2.waitKey(5) & 0xFF
                    # msg_right = bridge.cv2_to_imgmsg(cv2.resize(frame_right, (240,180), interpolation=cv2.INTER_NEAREST), "bgr8")
                    pub_right.publish(msg_right)
                    
                    
                    if rospy.is_shutdown():
                        break
                    # if rospy.is_shutdown():
                    #     break

        finally:
            cap_left.stop()
            cap_left.release()
            cap_right.stop()
            cap_right.release()
    else:
        cap_left.stop()
        cap_left.release()
        cap_right.stop()
        cap_right.release()

if __name__ == "__main__":
    try:
        ball_stereo_detection_pub()
    except rospy.ROSInterruptException:
        pass

