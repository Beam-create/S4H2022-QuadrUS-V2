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
