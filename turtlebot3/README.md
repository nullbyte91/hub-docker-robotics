# Docker for Turtlebot3 RL training using Gazebo
```bash
# Install docker [Host]
bash install_nvidia_docker.bash

# build docker image [Host]
docker build -t kinetic:v1 -f kinetic.Dockerfile .
bash run-nvidia.sh

# Install dep [Docker]
source /home/.anaconda2/bin/activate
bash ~/install_turtlebot3.bash

# Bug fix/changes [Docker]
#Turtlebot3’s LDS default is set to 360 modify to 24
sudo vim /opt/ros/kinetic/share/turtlebot3_description/urdf/turtlebot3_burger.gazebo.xacro +111

# TRaining dim error [Docker]
# Change state_size = 26 to state_size = 326
cd ~/catkin_ws/
vim src/turtlebot3_machine_learning/turtlebot3_dqn/nodes/turtlebot3_dqn_stage_1 +151

# Build package [Docker]
catkin_make

# RL training [Docker]
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch &
roslaunch turtlebot3_dqn turtlebot3_dqn_stage_1.launch

# Once the testing is done, Commit the docker changes from host
# Docker commit [Host]
docker ps #return image ID
docker image_ID kinetic:v1 

# Enable SSH
sudo vim /etc/ssh/sshd_config # Disable PermitRootLogin
```