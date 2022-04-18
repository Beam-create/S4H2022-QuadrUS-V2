# Fonctionnement du package

Le package rufus_remote doit être présent dans l'ordinateur connecté au master
Ce package contient les scripts pour le contrôle de la base et du bras, et le launch file pour se connecter au robot

## Nodes:

Le package contient le cde source des nodes suivantes:
- base_teleop
- bras_teleop

## Launch files

Le launch file pour connecter l'ordinateur au robot est:
rufus_remote.launch

La commande pour partir le laucnh file est:

    roslaunch rufus_remote rufus_remote.launch
