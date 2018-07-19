#!/usr/bin/env bash

set -e

#sudo cp files/49-teensy.rules /etc/udev/rules.d/

BASE="2wd"
SENSOR="x4"

ROBOT_WS="ros_education_ws"

ROSDISTRO="$(rosversion -d)"
#BASE=$1
#SENSOR=$2
ARCH="$(uname -m)"
echo $ARCH
echo "
______ _____________   _________ ________ _______ ________ _______ ________
___  / ____  _/___  | / /__  __ \___  __ \__  __ \___  __ )__  __ \___  __/
__  /   __  /  __   |/ / _  / / /__  /_/ /_  / / /__  __  |_  / / /__  /   
_  /_____/ /   _  /|  /  / /_/ / _  _, _/ / /_/ / _  /_/ / / /_/ / _  /    
/_____//___/   /_/ |_/   \____/  /_/ |_|  \____/  /_____/  \____/  /_/     
                    
                            http://linorobot.org                                                                          
"
if [ "$3" != "test" ]
    then
: 'Not used
        if [ "$*" == "" ]
            then
                echo "No arguments provided"
                echo
                echo "Example: $ ./install.sh 2wd xv11"
                echo
                exit 1
                
        elif [[ "$1" != "2wd" && "$1" != "4wd" && "$1" != "mecanum" && "$1" != "ackermann" ]]
            then
                echo "Invalid linorobot base: $1"
                echo
                echo "Valid Options:"
                echo "2wd"
                echo "4wd"
                echo "ackermann"
                echo "mecanum"
                echo
                exit 1

        elif [[ "$2" != "xv11" && "$2" != "rplidar" && "$2" != "kinect" && "$2" != "realsense" ]]
            then
                echo "Invalid linorobot base: $2"
                echo
                echo "Valid Options:"
                echo "xv11"
                echo "rplidar"
                echo "kinect"
                echo "realsense"
                echo
                exit 1
        
        elif [[ "$ARCH" != "x86_64" && "$2" == "realsense" ]]
            then
                echo "Intel Realsense R200 is not supported in $ARCH architecture."
                exit 1

        fi
'

        echo
        echo -n "You are installing ROS-$ROSDISTRO Linorobot for $BASE base with a $SENSOR sensor. Enter [y] to continue. " 
        read reply
        if [[ "$reply" != "y" && "$reply" != "Y" ]]
            then
                echo "Wrong input. Exiting now"
                exit 1
        fi
fi

echo
echo "INSTALLING NOW...."
echo

: 'Not used
sudo apt-get update
sudo apt-get install -y avahi-daemon
sudo apt-get install -y openssh-server
sudo apt-get install -y python-setuptools 
sudo apt-get install -y python-dev 
sudo apt-get install -y build-essential
sudo apt-get install -y python-gudev

sudo easy_install pip
sudo pip install -U platformio
sudo rm -rf $HOME/.platformio/
'

source /opt/ros/$ROSDISTRO/setup.bash

cd $HOME
mkdir -p $ROBOT_WS/src
cd $HOME/$ROBOT_WS/src
catkin_init_workspace
git clone https://github.com/linorobot/lino_msgs.git

cd $HOME/$ROBOT_WS/
catkin_make
source devel/setup.bash

: 'Not used
sudo apt-get install -y ros-$ROSDISTRO-roslint
sudo apt-get install -y ros-$ROSDISTRO-rosserial
sudo apt-get install -y ros-$ROSDISTRO-rosserial-arduino
sudo apt-get install -y ros-$ROSDISTRO-imu-filter-madgwick
sudo apt-get install -y ros-$ROSDISTRO-gmapping
sudo apt-get install -y ros-$ROSDISTRO-map-server
sudo apt-get install -y ros-$ROSDISTRO-navigation
sudo apt-get install -y ros-$ROSDISTRO-robot-localization

if [[ "$3" == "test" ]]
    then
        sudo apt-get install -y ros-$ROSDISTRO-xv-11-laser-driver
        sudo apt-get install -y ros-$ROSDISTRO-freenect-launch
        sudo apt-get install -y ros-$ROSDISTRO-realsense-camera
        sudo apt-get install -y ros-$ROSDISTRO-depthimage-to-laserscan
        sudo apt-get install -y ros-$ROSDISTRO-teb-local-planner

        cd $HOME/$ROBOT_WS/src
        git clone https://github.com/robopeak/rplidar_ros.git

else
    if [[ "$SENSOR" == "xv11" ]]
        then
            sudo apt-get install -y ros-$ROSDISTRO-xv-11-laser-driver
            
    elif [[ "$SENSOR" == "kinect" ]]
        then
            sudo apt-get install -y ros-$ROSDISTRO-freenect-launch
            sudo apt-get install -y ros-$ROSDISTRO-depthimage-to-laserscan

    elif [[ "$SENSOR" == "rplidar" ]]
        then
            cd $HOME/$ROBOT_WS/src
            git clone https://github.com/robopeak/rplidar_ros.git

    elif [[ "$SENSOR" == "realsense" ]]
        then
            sudo apt-get install -y ros-$ROSDISTRO-realsense-camera
            sudo apt-get install -y ros-$ROSDISTRO-depthimage-to-laserscan
    fi

    if [[ "$BASE" == "ackermann" ]]
        then
            sudo apt-get install -y ros-$ROSDISTRO-teb-local-planner
    fi
fi
'

cd $HOME/$ROBOT_WS/src
#git clone https://github.com/linorobot/linorobot.git
git clone https://github.com/linorobot/imu_calib.git
git clone https://github.com/linorobot/lino_pid.git
git clone https://github.com/linorobot/lino_udev.git
git clone https://github.com/linorobot/lino_visualize.git

git clone https://github.com/EAIBOT/ydlidar.git
git clone https://github.com/ROS-education/linorobot.git

cd $HOME/$ROBOT_WS/src/linorobot
TRAVIS_BRANCH="echo $TRAVIS_BRANCH"
if [ "$TRAVIS_BRANCH" = "devel" ]; then git checkout devel; fi

: 'Not used
rm -rf $HOME/$ROBOT_WS/src/linorobot/teensy/firmware/lib/ros_lib/lino_msgs
rosrun rosserial_arduino make_libraries.py /tmp
cp -r /tmp/ros_lib/lino_msgs/ $HOME/$ROBOT_WS/src/linorobot/teensy/firmware/lib/ros_lib/

cd $HOME/$ROBOT_WS/src/linorobot/teensy/firmware
export PLATFORMIO_CI_SRC=$PWD/src/firmware.ino
platformio ci --project-conf=./platformio.ini --lib="./lib/ros_lib" --lib="./lib/config"  --lib="./lib/motor"  --lib="./lib/kinematics"  --lib="./lib/pid"  --lib="./lib/imu" --lib="./lib/encoder"
'

echo "source $HOME/$ROBOT_WS/devel/setup.bash" >> $HOME/.bashrc
echo "export LINOLIDAR=$SENSOR" >> $HOME/.bashrc
echo "export LINOBASE=$BASE" >> $HOME/.bashrc
source $HOME/.bashrc

cd $HOME/$ROBOT_WS
catkin_make

echo
echo "INSTALLATION DONE!"
echo

: 'After installation, you need ROOT user to install these udev rules
roscd ydlidar/startup
sudo chmod 777 ./*
sudo sh initenv.sh

'
