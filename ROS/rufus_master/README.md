# Fonctionnement du package

Le package rufus_master doit être téléchargé dans le Jetson Nano. Ce package contient les scripts pour le contrôle de la base et le launch file pour démarer le robot

## Messages:

Le package crée les messages suivants:

-   bras_commands.msg
-   Feedback_arduino_msgs.msg
-   Rufus_base_msgs.msg

## Nodes:

Le package contient le cde source des nodes suivantes:

-   base_control_node

## Launch files

Le launch file pour démarrer le robot est:
rufus_master.launch

La commande pour partir le laucnh file est: 

    roslaunch rufus_master rufus_master.launch
