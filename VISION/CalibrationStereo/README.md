# Calibration de la caméra stéréoscopique
 1. Se rendre dans le répertoire **CalibrationStereo**
	`cd CalibrationStereo`
## Acquisition
Afin de calibrer la caméra stéréoscopique, on doit utiliser un échiquier de calibration.
Après l'impression sur une feuille 8½x11, on puet exécuter la calibration avec :
   ` python3 Acquisition.py`
Afin d'enregistrer une image, on appuie sur la touche `ESPACE`, pour quitter l'application, on appuie sur la touche `q` ou `ESC`.

## Calibration
Lorsque la totalité du champ de vision de la caméra a été couvert par des photos de l'échiquier, on puet poursuivre avec la calibration.
La calibration consiste à premièrement rectifier les images individuellement afin d'éliminer la distorsion causée par la lentille, puis à calibrer la stéréoscopie.
Il suffit d'exécuter :

    python3 StereoCalibration.py
Les données de calibration seront ainsi enregistrées sous **stereoMap.xml**.

## Test de la calibration
Afin d'évaluer la qualité de la calibration, on peut exécuter le programme **StereoVision_RectificationTest.py**.
`python3 StereoVision_RectificationTest.py`

Si la calibration est erronée, on peut refaire l'acquisition. Le processus est très itératif.
   
