#!/usr/bin/env python3
import sys
import cv2
import numpy as np
import time


def add_HSV_filter(frame, camera):

	# Ajout d'un flou sur l'image - aide a reduire le bruit et a uniformiser les couleurs
    blur = cv2.GaussianBlur(frame,(5,5),0) 

    # Conversion RGB a HSV
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

    l_b_r = np.array([0, 130, 126])        # Lower bound
    u_b_r = np.array([24, 255, 255])      # Upper bound 
    l_b_l = np.array([0, 130, 126])
    u_b_l = np.array([24, 255, 255])       


    if(camera == 1):
        mask = cv2.inRange(hsv, l_b_r, u_b_r)
    else:
        mask = cv2.inRange(hsv, l_b_l, u_b_l)


    # Operation afin de reduire le bruit present dans le masque 
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    return mask
