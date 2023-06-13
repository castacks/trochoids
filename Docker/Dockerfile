FROM ros:melodic

WORKDIR /ws/src

WORKDIR /ws
RUN apt-get update 
RUN apt-get install python3-catkin-tools ros-melodic-angles ros-melodic-tf libeigen3-dev -y 
RUN apt-get install libbenchmark-dev -y
RUN catkin config --extend /opt/ros/$ROS_DISTRO 
RUN catkin build 


# Bitbucket build constraints
# docker build --memory=1g --memory-swap=1g -t bit1 -f my.dockerfile .
# docker build --memory=8g --no-cache -t bradygm/trochoids-test -f Docker/Dockerfile .

# Bitbucket run constraints
# docker run -it --memory=4g --memory-swap=4g --memory-swappiness=0 --cpus=4 --entrypoint=/bin/bash bit1:latest




