#!/usr/bin/env python3
#Fonction qui va gerer les limites

import json

def verify_limits(array, mode):
    lim = json.load(open('/home/projetS4/src/S4H2022-projet/ROS/rufus_master/lib/Limits.JSON', "r"))
    is_good_angle = True
    is_good_position = True

    if mode: #Position x,y,z dans array (Inverse kinematic)
        #array[0] = x ; array[1] = y ; array[2] = z
        if array[0] <= lim['x']['min'] or array[0] >= lim['x']['max'] or array[1] <= lim['y']['min'] or array[1] >= lim['y']['max'] or array[2] <= lim['z']['min'] or array[2] >= lim['z']['max']:
            is_good_position = False
            return is_good_position
        else:
            return is_good_position
    else: #Verification des valeurs d'angle dans l'array
        #array[0] = q1 ; array[1] = q2 ; array[2] = q3
        if array[0] <= lim['q1']['min'] or array[0] >= lim['q1']['max'] or array[1] <= lim['q2']['min'] or array[1] >= lim['q2']['max'] or array[2] <= lim['q3']['min'] or array[2] >= lim['q3']['max']:
            is_good_angle = False
            return is_good_angle
        else:
            return is_good_angle
            