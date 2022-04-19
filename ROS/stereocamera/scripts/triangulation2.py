#!/usr/bin/env python3
import sys
import cv2
import numpy as np
import time


def find_depth_and_angle(right_point, left_point, frame_right, frame_left, baseline,f, alpha):

    # # Convertir la distance focal en pixel [mm]->[pixel]:
    height_right, width_right, depth_right = frame_right.shape
    height_left, width_left, depth_left = frame_left.shape
    half_width = width_left/2
    # if width_right == width_left:  
    #     
    # else:
    #     print('Erreur de lecture des images : les dimensions ne concordent pas...')

    x_right = right_point[0]
    x_left = left_point[0]

    # Calcul de la disparite (entre pixels)
    disparity = x_left-x_right     
    zDepth = -50
    if disparity != 0:
        angle_dx = x_left+x_right-width_right
        # Calcul de la profondeur en cm linearise:

        zDepth = disparity*0.0270 - 25.519 + 9130/disparity        
        angle_deg = angle_dx*((alpha/2)/half_width)*0.5351
        angle_rad = angle_deg*np.pi/180
        x_cart = np.sin(angle_rad)*zDepth
        z_cart = np.cos(angle_rad)*zDepth
    return zDepth, angle_deg, x_cart, z_cart
