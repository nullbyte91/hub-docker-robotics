#!/bin/bash

# Copyright (c) 2021 Jegathesan Shanmugam
# Released under the MIT License (MIT)
# https://github.com/nullbyte91/Simple-Sh-DataScience/blob/master/LICENSE.md

# Title           : install_turtlebot3.bash
# Description     : This script use to setup turtlebot3 for ml(https://emanual.robotis.com/docs/en/platform/turtlebot3/machine_learning/)
# author		  : Jegathesan Shanmugam

function dep_install()
{
    rosdep update
    python -m pip install --upgrade pip --user

    # ROS Dep
    python -m pip install msgpack argparse rosinstall empy defusedxml netifaces --user
    python -m pip install tensorflow-gpu==1.15.0 --user
    python -m pip install keras==2.1.5 --user
    python -m pip install tensorboard --user

    # Install turtlebot3 dep
    sudo apt-get update && sudo apt-get install -y ros-kinetic-turtlebot3*

    # Uninstall from both conda and main env
    python -m pip uninstall -y numpy 
    python -m pip uninstall -y numpy

    python -m pip install numpy pyqtgraph --user
}

function turtlebot3_ml()
{
    # Machine learning packages
    mkdir -p ~/catkin_ws/src/
    cd ~/catkin_ws/src/
    git clone https://github.com/ROBOTIS-GIT/turtlebot3_machine_learning.git
    cd ~/catkin_ws && catkin_make

    cd ~/catkin_ws/src/
    git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    cd ~/catkin_ws && catkin_make
}

# Main starts from here
echo "######## Setup start #############"
dep_install
turtlebot3_ml
echo "######## Setup done #############"

