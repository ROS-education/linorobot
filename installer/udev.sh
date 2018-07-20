#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ~/ros_education_ws/devel/setup.bash

#maple mini
#ATTRS{idProduct}=="0003", ATTRS{idVendor}=="1eaf", MODE="664", GROUP="plugdev" SYMLINK+="maple"
#ATTRS{idProduct}=="0004", ATTRS{idVendor}=="1eaf", MODE="664", GROUP="plugdev" SYMLINK+="maple"

echo  'ATTRS{idVendor}=="1eaf", ATTRS{idProduct}=="0003", MODE:="0666", GROUP:="dialout",  SYMLINK+="linobase"' >/etc/udev/rules.d/46-maple_mini.rules

#ydlidar
roscd ydlidar/startup
sudo chmod 777 ./*
sudo sh initenv.sh



