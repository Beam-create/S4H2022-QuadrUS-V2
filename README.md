
# S4H2022-RufUS

### Robot-chien | UdeS-GRO
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg?style=flat-square)](http://makeapullrequest.com) [![Open Source Love](https://badges.frapsoft.com/os/v1/open-source.svg?v=103)](https://github.com/ellerbrock/open-source-badges/)[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

## Téléchargement du projet
1. Installez le fichier d'installation grâce à la commande ci-dessous:
 `wget https://raw.githubusercontent.com/Beam-create/S4H2022-RufUS/main/Installation.sh`
2. Exécutez le fichier grâce à cette commande:
 `bash Installation.sh`

> L'exécution du fichier permet d'installer automatiquement toutes les librairies ROS et les dépendances nécessaires au projet. Il permet aussi de configurer l'infrastructure ROS du projet. En effet, Un dossier sous le nom de rufus_ws (rufus workspace) contenant tous les éléments nécessaire au fonctionnement de la plateforme sera créé.

## Présentation du projet
![image](https://user-images.githubusercontent.com/72213923/155257730-0c8ef9f5-0139-4f08-8084-f2a888d273e7.png)

RufUS est un robot collaboratif conçu et fabriqué par 6 étudiants en génie robotique à l'Université de Sherbrooke. Ce projet est fait dans le cadre du projet de la session 4.

Le but de ce projet est de construire un robot ayant la capacité de détecter, saisir puis rapporter un item. Pour ce faire, RufUS intègre une vision caméra, un bras robotique et une base mobile.



## Table des matières wiki
* [Documentation](#documentation)
	* [Base](https://github.com/Beam-create/S4H2022-RufUS/wiki/Base-mobile)
		* [Liste des matériaux et composantes](https://github.com/Beam-create/S4H2022-RufUS/wiki/Base-mobile#liste-des-mat%C3%A9riaux-et-composantes)
		* [Assemblage mécanique](https://github.com/Beam-create/S4H2022-RufUS/wiki/Base-mobile#assemblage-m%C3%A9canique)
		* [Assemblage électrique](https://github.com/Beam-create/S4H2022-RufUS/wiki/Base-mobile#assemblage-%C3%A9lectrique)
		* [Préparation logicielle](https://github.com/Beam-create/S4H2022-RufUS/wiki/Base-mobile#pr%C3%A9paration-logicielle)
	* [Bras](https://github.com/Beam-create/S4H2022-RufUS/wiki/Bras-Robot)
		* [Assemblage du bras robot](https://github.com/Beam-create/S4H2022-RufUS/wiki/Bras-Robot#assemblage-du-bras-robot)
		* [Contrôle du bras robot](https://github.com/Beam-create/S4H2022-RufUS/wiki/Bras-Robot#contr%C3%B4le-du-bras-robot)
		* [Dépendances logicielles](https://github.com/Beam-create/S4H2022-RufUS/wiki/Bras-Robot#d%C3%A9pendances-logicielles)
	* [Vision](https://github.com/Beam-create/S4H2022-RufUS/wiki/Vision)
		* 
	* [Configuration du Jetson Nano](https://github.com/Beam-create/S4H2022-RufUS/wiki/Configuration-du-Jetson-Nano-B01)
		* [Installation de Ubuntu 20.04](https://github.com/Beam-create/S4H2022-RufUS/wiki/Configuration-du-Jetson-Nano-B01#installation-de-ubuntu-2004)
		* [Création d'un hotspot](https://github.com/Beam-create/S4H2022-RufUS/wiki/Configuration-du-Jetson-Nano-B01#cr%C3%A9ation-dun-hotspot)
		* [Connexion au Jetson Nano par SSH](https://github.com/Beam-create/S4H2022-RufUS/wiki/Configuration-du-Jetson-Nano-B01#connexion-au-jetson-nano-par-ssh)
* [Licence](https://github.com/Beam-create/S4H2022-RufUS/blob/main/README.md#licence-mit)

## Documentation
### Base:

Les pièces de la base se trouvent dans ce [dossier](https://drive.google.com/drive/folders/1wTbaxu6NTdUD7D1Q0fb7H7a6KUzrX4lB?usp=sharing). L'assemblage [électrique](https://github.com/Beam-create/S4H2022-RufUS/wiki/Base-mobile#assemblage-électrique) requiert un accès à des équipements spécialisés et une connaissance de base de l'électronique. 
### Bras:
En ce qui concerne les pièces du bras, nous avons modifié un modèle déjà existant dont tous les cads se trouvent [ici](https://drive.google.com/drive/folders/1wTbaxu6NTdUD7D1Q0fb7H7a6KUzrX4lB?usp=sharing). Pour l'assemblage,  se référer à cette [page](https://www.instructables.com/EEZYbotARM-Mk2-3D-Printed-Robot/) rédigé par *theGHIZmo*.

### Vision:
**Caméra stéréoscopique:**   [IMX219-83](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera)


### Licence MIT
Copyright 2022 RufUS

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
