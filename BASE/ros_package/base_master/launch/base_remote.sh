#!/bin/bash

export ROS_MASTER_URI=http://192.168.1.14:11311/
export ROS_IP="192.168.1.11"
export ROSLAUNCH_SSH_UNKNOWN=1

source /opt/ros/noetic/setup.bash

exec "$@"