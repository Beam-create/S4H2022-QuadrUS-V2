# S4H2022-RufUS/BRAS

## Assemblage du bras robot

[Instructions de montage du bras.pdf](https://github.com/Beam-create/S4H2022-RufUS/files/8430149/Instructions.de.montage.du.bras.pdf)

Voici le site sur lequel ces instructions sont basés : [lien](http://www.eezyrobots.it/eba_mk2.html)

Tout crédit pour le modèle utilisé se réserve à: Carlo Franciscone, daGHIZmo ([profil thingiverse]( https://www.thingiverse.com/daghizmo/designs)).

Certaines modifications au niveau du dimensionnement ont été apportés afin de répondre au besoin du projet RufUS: 
 1- Assise de servo moteur.
 2- Longueur des membrures.

Lien pour la license CC: https://www.thingiverse.com/thing:1454048

## Contrôle du bras robot

### Cinématique inverse
Afin de se rendre à position cartésienne définie par la position de la balle (position détecté par la caméra), une cinématique inverse a été modélisée à partir des schéma suivant :

1- Vue de coter du bras du robot :
![vue_coter2](https://user-images.githubusercontent.com/72227713/163698267-9b8c015c-d479-4254-9cab-0b8b52da7252.PNG)

2- Vue de dessus du bras du robot : 
![vue_dessus2](https://user-images.githubusercontent.com/72227713/163698269-1dca8a0d-97f3-4ff2-9430-3ec72d58ef7c.PNG)

Les équations tirés de ces schémas se trouve dans bras_teleop.py (https://github.com/Beam-create/S4H2022-RufUS/blob/main/ROS/rufus_remote/src/bras_teleop.py). Avec la position en "x" et en "z" de la balle, il est possible de déterminer l'angle de rotation du joint 1. Par la suite, une fois que l'angle 1 est connue, il est possible d'utiliser le "solver" "nsolve" de python pour résoudre le système à deux équations et 2 inconnues pour trouver la valeur des angles pour les joints 2 et 3. Les valeurs de 0, pi/2, 0 représente les valeurs des angles initiales en radians des joints 1, 2 et 3 respectivement, ce qui permet d'accélérer la résolution du système d'équation

### Calibration des servo
Afin de bien modéliser le comportement réel des servos moteurs, une calibration est utilisée dans le but d'extraire une fonction approximative du signal PWM en fonction de l'angle désiré. 

Les étapes de calibration sont :

 1- Création d'une série de donnée [angles_réels, PWM_envoyés] pour chaque servo.

 2- Selon l'allure de la courbe, on utilise une fonction de linéarisation à plusieurs coefficients afin d'approximer la courbe de données discrète. 

 3- Mesuré les erreurs quadratiques afin d'assurer une précision adéquate.

 4- Modéliser la fonction de writeMicroseconds selon la fonction linéarisée obtenue.

*Pour ce faire, Matlab/Python sont de bons outils d'analyse numérique. 

## Dépendances logicielles
Pour le fonctionnement bas-niveau du bras, le code c++ doit inclure les librairies externes suivantes:

1- Servo.h, librairie Arduino pour le contrôle de servo moteur selon une commande PWM ou un angle. Lien de téléchargement: https://github.com/arduino-libraries/Servo

2- Ramp.h, librairie d'interpolation de donnée afin d'adoucir les mouvements du bras robot.
