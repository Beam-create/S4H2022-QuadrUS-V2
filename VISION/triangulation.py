import sys
import cv2
import numpy as np
import time


def find_depth(right_point, left_point, frame_right, frame_left, baseline,f, alpha):

    # Convertir la distance focal en pixel [mm]->[pixel]:
    height_right, width_right, depth_right = frame_right.shape
    height_left, width_left, depth_left = frame_left.shape

    if width_right == width_left:
        f_pixel = (width_right * 0.5) / np.tan(alpha * 0.5 * np.pi/180)

    else:
        print('Erreur de lecture des images : les dimensions ne concordent pas...')

    x_right = right_point[0]
    x_left = left_point[0]

    # Calcul de la disparite (entre pixels)
    disparity = x_left-x_right      

    # Calcul de la profondeur en cm:
    zDepth = (baseline*f_pixel)/disparity             

    return zDepth

