

# S4H2022-RufUS

### Robot-chien | UdeS-GRO
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](http://makeapullrequest.com) [![Open Source Love](https://badges.frapsoft.com/os/v1/open-source.svg?v=103)](https://github.com/ellerbrock/open-source-badges/)[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Présentation du projet
<p align="center">
    	<img src="https://user-images.githubusercontent.com/54538310/163730728-76d62d03-951f-4c52-9c64-2c97f2e0f997.jpg" />
    	<br>
    	<img src="https://user-images.githubusercontent.com/54538310/163886041-4ef157aa-282a-4b21-9bb6-5ed436bbb83b.jpg" />
	<br>
	<a>Respectivement <strong>Antoine Waltz</strong>, <strong>Jordan Simard</strong>, <strong>Guillaume Blain</strong>, <strong>Mathieu Beaudoin</strong>, <strong>Christopher Pacheco</strong> et <strong>Nicolas Longchamps</strong></a>
</p>

RufUS est un robot collaboratif conçu et fabriqué par 6 étudiants en génie robotique à l'Université de Sherbrooke. Ce projet est fait dans le cadre du projet de la session 4.

Le but de ce projet est de construire un robot ayant la capacité de détecter, saisir puis rapporter un item. Pour ce faire, RufUS intègre une vision caméra, un bras robotique et une base mobile.


## Téléchargement du projet
1. Installez le fichier d'installation grâce à la commande ci-dessous:
 ```
 wget https://raw.githubusercontent.com/Beam-create/S4H2022-RufUS/main/Installation.sh
 ```
2. Exécutez le fichier grâce à cette commande:
 ```
 bash Installation.sh
 ```

> L'exécution du fichier permet d'installer automatiquement toutes les librairies ROS et les dépendances nécessaires au projet. Il permet aussi de configurer l'infrastructure ROS du projet. En effet, Un dossier sous le nom de rufus_ws (rufus workspace) contenant tous les éléments nécessaire au fonctionnement de la plateforme sera créé.
### Validation de l'installation
1. Ouvrir un terminal et exécutez cette commande:
```
gedit .bashrc
```
2. Allez à la fin du fichier
3. Validez que *source ~/rufus_ws/devel/setup.bash* ne se trouver par entre ( " " )
## Comment démarrer le projet
1. Suivre les instructions d'installation sur un ordinateur avec Ubuntu20.04.
2. Suivre les instructions d'installation de Ubuntu20.04 sur le Jetson Nano (Voir [Configuration du Jetson Nano](/VISION#configuration-du-jetson-nano))
3. Lorsque l'installation est terminée sur le Jetson Nano, configurez le hotspot (Voir [Création d'un hotspot](/ROS#création-dun-hotspot)).
4. Avec le hotspot configuré, connectez vous par SSH au Jetson Nano.
5. Une fois connecté, allez dans le dossier rufus_ws et exécutez une compilation du programme ROS :
```
cd ~/rufus_ws     
catkin_make
```
puis une fois compilé,
```
source devel/setup.bash
```
6. Finalement, toujours sur le Jetson Nano avec la connexion SSH, lancez le programme *master* :
```
roslaunch rufus_master rufus_master.launch
```
7. Finalement, avec l'ordinateur *remote*, lancez :
```
roslaunch rufus_remote rufus_remote.launch
```
Et le tour est joué!
**Troubleshooting**
Si des problèmes de connexion ont lieu, vérifiez que les adresses IP concordent avec les *launchfiles*.

## Table des matières
* Documentation
	* [Base](/FABRICATION/BASE#s4h2022-rufusbase)
		* [Fichiers CAD](/FABRICATION/BASE#fichers-cad)
		* [Comment fabriquer la base mobile](/FABRICATION/BASE#comment-fabriquer-la-base-mobile) 
			* [Liste des matériaux et composantes](/FABRICATION/BASE#liste-des-matériaux-et-composantes)
			* [Assemblage mécanique](/FABRICATION/BASE#assemblage-mécanique)
			* [Assemblage électrique](/FABRICATION/BASE#assemblage-électrique)
	* [Bras](/BRAS.md)
		* [Assemblage du bras robot](/BRAS.md#assemblage-du-bras-robot)
		* [Contrôle du bras robot](/BRAS.md#contrôle-du-bras-robot)
		* [Dépendances logicielles](/BRAS.md#dépendances-logicielles)
	* [Vision](/VISION#s4h2022-rufusvision)
		* [Matériel nécessaire](/VISION#matériel-nécessaire)
		* [Configuration du Jetson Nano](/VISION#configuration-du-jetson-nano)
			* [Installation de Ubuntu 20.04](/VISION#installation-de-ubuntu-2004)
			* [Installation des dépendances (vision)](/VISION#installation-des-dépendances)  
	* [Arduino](/ARDUINO#s4h2022-rufusarduino)
	* [ROS](/ROS#s4h2022-rufusros)
		* [Configuration du Jetson Nano](/ROS#configuartion-du-jetson-nano) 
			* [Création d'un hotspot](/ROS#création-dun-hotspot)
			* [Connexion par ssh](/ROS#connexion-au-jetson-nano-par-ssh) 
* [Licence](https://github.com/Beam-create/S4H2022-RufUS/blob/main/README.md#licence-mit)


### Licence MIT
Copyright 2022 RufUS

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
