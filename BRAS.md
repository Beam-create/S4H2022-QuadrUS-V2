
# S4H2022-RufUS/BRAS
## Liste des matériaux et composantes
- [ ] 1 x Servo DS3225  ([amazon](https://www.amazon.ca/-/fr/enti%C3%A8rement-m%C3%A9tallique-num%C3%A9rique-voitures-contr%C3%B4le/dp/B07NSG5WFQ/ref=sr_1_7?__mk_fr_CA=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=250SI05Q6DGX1&keywords=servo+25+kg&qid=1650227407&sprefix=servo+25kg%2Caps%2C165&sr=8-7))
- [ ] 2 x Servo DS3218 ([amazon](https://www.amazon.ca/Servomoteur-num%C3%A9rique-%C3%A9tanche-voiture-contr%C3%B4le/dp/B07MDM1C3M/ref=sr_1_1_sspa?gclid=Cj0KCQjwmPSSBhCNARIsAH3cYgbY-I6gcLQd_m05WRdz0jvEnY4g75Xh6RnTGzBoZARoA3yt9WMbEREaAghcEALw_wcB&hvadid=231054272123&hvdev=c&hvlocphy=9000516&hvnetw=s&hvqmt=e&hvrand=12183318026730068377&hvtargid=kwd-297225592226&hydadcr=20845_10090782&keywords=ds3218&qid=1650316875&sr=8-1-spons&psc=1&spLa=ZW5jcnlwdGVkUXVhbGlmaWVyPUEyVVdaMk1HOUFRQ0E4JmVuY3J5cHRlZElkPUEwOTA3MDA2M0IyWUowSktWRU9POSZlbmNyeXB0ZWRBZElkPUEwMDYwNjI5M0ZVTEVDS0dSNEtZUSZ3aWRnZXROYW1lPXNwX2F0ZiZhY3Rpb249Y2xpY2tSZWRpcmVjdCZkb05vdExvZ0NsaWNrPXRydWU=))
- [ ] 1 x Servo SG90 ([amazon](https://www.amazon.ca/-/fr/servomoteur-radiocommand%C3%A9-h%C3%A9licopt%C3%A8re-v%C3%A9hicules-compatible/dp/B097RD8RB7/ref=sr_1_16?__mk_fr_CA=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=BJCQ5BYGP5O8&keywords=servo+9+g&qid=1650227478&sprefix=servo+9g%2Caps%2C74&sr=8-16))
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
Afin de se rendre à la position cartésienne de la balle (position détecté par la caméra), une cinématique inverse a été modélisée à partir des schéma suivant :

1- Vue de coter du bras du robot :
![vue_coter2](https://user-images.githubusercontent.com/72227713/163698267-9b8c015c-d479-4254-9cab-0b8b52da7252.PNG)

2- Vue de dessus du bras du robot : 
![vue_dessus2](https://user-images.githubusercontent.com/72227713/163698269-1dca8a0d-97f3-4ff2-9430-3ec72d58ef7c.PNG)

Les équations tirés de ces schémas sont :
<p align="center">
    <img src="https://latex.codecogs.com/svg.image?q_1=tan^{-1}(\frac{Z}{X})" height="60" />
    <br>
    <img src="https://latex.codecogs.com/svg.image?x=cos(q_1)*(L_2cos(q_2)+L_3cos(q_3)+L__{4x})-cam_x"  height="30"/>
    <br>
    <img src="https://latex.codecogs.com/svg.image?y=L_1+L_2sin(q_2)-L_3sin(q_3)-L__{4y}+cam_y"  height="30"/>
</p>

Avec la position en "x" et en "z" de la balle, il est possible de déterminer l'angle de rotation du joint 1. Par la suite, une fois que l'angle 1 est connu, il est possible d'utiliser le "solver" "nsolve" de python pour résoudre le système à deux équations et 2 inconnues pour trouver la valeur des angles pour les joints 2 et 3. Les valeurs de pi/2 et 0 dans le "nsolve" représente les valeurs des angles initiales en radians des joints 2 et 3 respectivement, ce qui permet d'accélérer la résolution du système d'équation.

### Calibration des servo
Afin de bien modéliser le comportement réel des servos moteurs, une calibration est utilisée dans le but d'extraire une fonction approximative du signal PWM en fonction de l'angle désiré. 

Les étapes de calibration sont :

 1- Création d'une série de données [angles_réels, PWM_envoyés] pour chaque servo.

 2- Selon l'allure de la courbe, on utilise une fonction de linéarisation à plusieurs coefficients afin d'approximer la courbe de données discrètes. 

 3- Mesurer les erreurs quadratiques afin d'assurer une précision adéquate.

 4- Modéliser la fonction de *writeMicroseconds()* selon la fonction linéarisée obtenue.

> Pour ce faire, Matlab/Python sont de bons outils d'analyse numérique. 

## Dépendances logicielles
Pour le fonctionnement bas-niveau du bras, le code c++ doit inclure la librairie externe suivante:

1- [Servo.h](https://github.com/arduino-libraries/Servo), librairie Arduino pour le contrôle de servo moteur selon une commande PWM ou un angle.

