

# S4H2022-RufUS

### Robot-chien | UdeS-GRO
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](http://makeapullrequest.com) [![Open Source Love](https://badges.frapsoft.com/os/v1/open-source.svg?v=103)](https://github.com/ellerbrock/open-source-badges/)[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Présentation du projet
![render_rufus](https://user-images.githubusercontent.com/54538310/163730728-76d62d03-951f-4c52-9c64-2c97f2e0f997.jpg)


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

## Comment initialiser le projet (A REVOIR)
1. Ouvrir un terminal
2. Écrire la commande :
```
gedit .bashrc
```
3. Aller au à la fin du fichier
4. Valider que *source ~/rufus_ws/devel/setup.bash* ne se trouver par entre (" ")

## Table des matières
* Documentation
	* [Base](/FABRICATION/BASE#s4h2022-rufusbase)
		* [Fichiers CAD](/BASE#fichers-cad)
		* [Comment fabriquer la base mobile](/BASE#comment-fabriquer-la-base-mobile) 
			* [Liste des matériaux et composantes](/BASE#liste-des-matériaux-et-composantes)
			* [Assemblage mécanique](/BASE#assemblage-mécanique)
			* [Assemblage électrique](/BASE#assemblage-électrique)
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
