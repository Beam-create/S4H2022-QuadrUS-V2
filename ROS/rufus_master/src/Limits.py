#!/usr/bin/env python3
#Fonction qui va gerer les limites
#ajout ligne pour push

lim = {
    "x_min":0.0,
    "x_max":20.0,
    "y_min":0.0,
    "y_max":20.0,
    "z_min":0.0,
    "z_max":20.0,
    
    "q1_min":-45.0,
    "q1_max":45.0,
    "q2_min":30.0,
    "q2_max":130.0,
    "q3_min":-15.0,
    "q3_max":60.0
}

def verify_limits(array, mode):
    is_good_angle = True
    is_good_position = True

    if mode: #Position x,y,z dans array (Inverse kinematic)
        #array[0] = x ; array[1] = y ; array[2] = z
        if array[0] <= lim['x_min'] or array[0] >= lim['x_max'] or array[1] <= lim['y_min'] or array[1] >= lim['y_max'] or array[2] <= lim['z_min'] or array[2] >= lim['z_max']:
            is_good_position = False
            return is_good_position
        else:
            return is_good_position
    else: #Verification des valeurs d'angle dans l'array
        #array[0] = q1 ; array[1] = q2 ; array[2] = q3
        if array[0] <= lim['q1_min'] or array[0] >= lim['q1_max'] or array[1] <= lim['q2_min'] or array[1] >= lim['q2_max'] or array[2] <= lim['q3_min'] or array[2] >= lim['q3_max']:
            is_good_angle = False
            return is_good_angle
        else:
            return is_good_angle
        