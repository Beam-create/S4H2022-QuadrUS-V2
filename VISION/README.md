# Configuration du Jetson Nano
## Installation de Ubuntu 20.04 
D'ici la sortie d'une version de JetPack (OS de base du Jetson Nano) utilisant Ubuntu 20.04.

Suivre le [tutoriel de QEngineering](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html) afin d'installer et de configurer le Jetson Nano.

**Une image précompilée (qui ne nécessite pas de "*build from source*") est disponible [ici](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image), par QEngineering également.**

On conseille l'image précompilée, qui ne prendra pas 6 heures de "*build*".
## Installation des dépendances
Pour les installations nécessaires (python3),

    pip3 install matplotlib numpy pillow imutils mediapipe 


### DualDetect.py
Ce programme permet de détecter la présence d'un visage et d'une balle de jaune ou orange. Ensuite, par triangulation, la distance est évaluée et renvoyée en cm sur l'image.

    python3 DualDetect.py

### test_vpi.py 
Ce programme évalue la disparité stéréoscopique et retourne une image de style *heatmap*. Il fonctionne pour le moment avec python2.7.

Afin de faire fonctionner la librairie VPI de NVIDIA pour l'utilisation de CUDA dans l'évaluation de la disparité, on doit créer un lien symbolique dans Ubuntu 20.04:

    cd /usr/lib/python2.7/dist-packages
    sudo ln -s /opt/nvidia/vpi1/lib64/python/vpi.so
Finalement, on doit modifier le fichier .bashrc afin d'éviter une erreur de dépendance :

**afin d'ouvrir le fichier .bashrc:**

    sudo nano ~/.bashrc

Ajouter à la fin du .bashrc :

    export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1
Sauvegarder et quitter le fichier. Le Jetson est maintenant prêt!
 

**Pour exécuter le programme test_vpi.py**

    python2 test_vpi.py
