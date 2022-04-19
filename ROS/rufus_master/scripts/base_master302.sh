#!/bin/bash

export ROS_MASTER_URI=http://10.244.35.87:11311/
export ROS_IP="10.244.35.87"
export ROSLAUNCH_SSH_UNKNOWN=1

source /opt/ros/noetic/setup.bash
source ~/rufus_ws/devel/setup.bash

echo "EXECUTED BASH FILE ON MASTER"

exec "$@"
