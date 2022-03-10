#!/usr/bin/env bash

echo "-----------------------------------------------"
echo "Installation des dépendances du projet rufus."
echo "Ne pas fermer le terminal avant la completion"
echo "-----------------------------------------------"

#ROS noetic, source : http://wiki.ros.org/noetic/Installation/Ubuntu
echo ""
echo "--> Installation de ROS noetic ..."
echo ""

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-get install curl -y || FAIL_ROS=true
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get update || FAIL_ROS=true
sudo apt-get install ros-noetic-desktop-full -y || FAIL_ROS=true
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

if $FAIL_ROS; then
    echo "Erreur d'installation de ROS"
    return
fi

echo ""
echo "--> Installation des packages supplémentaires ..."
echo ""
sudo apt install ros-noetic-joy || FAIL_PKG=true
sudo apt install ros-noetic-rosserial || FAIL_PKG=true
sudo apt install ros-noetic-rosserial-python || FAIL_PKG=true
sudo apt install ros-noetic-rosserial-arduino || FAIL_PKG=true

if $FAIL_PKG; then
    echo "Erreur d'installation des packages"
    return
fi

echo ""
echo "--> Installation de modules python ..."
echo ""
sudo apt-get install python3-pip -y || FAIL_PYTHON=true
sudo apt-get install python-is-python3 -y || FAIL_PYTHON=true
python -m pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose || FAIL_PYTHON=true

if $FAIL_PYTHON; then
    echo "Erreur d'installation des modules Python"
    return
fi

echo ""
echo "--> Création des dossiers du projet"
echo ""

mkdir -p ~/rufus_ws/src
cd ~/rufus_ws/src || exit 1
catkin_init_workspace || FAIL_REPO=true
git clone "https://github.com/Beam-create/S4H2022-RufUS.git" || exit 1
cd ~/rufus_ws || exit 1
catkin_make || FAIL_REPO=true
echo "source ~/rufus_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

if $FAIL_REPO; then
    echo "Erreur de création du dossier de projet"
    return
fi

echo "-----------------------------------------------"
echo "L'installation du projet rufus c'est fait avec "
echo "succès. Merci d'avoir téléchargé notre projet."
echo "Godspeed."
echo "-----------------------------------------------"