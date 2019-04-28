#!/bin/bash

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' &&
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 &&
sudo apt update &&
sudo apt install -y ros-melodic-desktop &&
sudo rosdep init &&
rosdep update &&
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc &&
source ~/.bashrc &&
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential &&

for prgname in kakasi kusakari mekaki
do
    mkdir -p ~/arcosaka/2019/${prgname}/src &&
    cd ~/arcosaka/2019/${prgname}/src &&
    catkin_init_workspace &&
    cd ~/arcosaka/2019/${prgname} &&
    catkin_make
done &&

echo "OK"