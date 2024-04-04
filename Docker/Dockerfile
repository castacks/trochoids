FROM ros:noetic

WORKDIR /ws/src

WORKDIR /ws
RUN apt-get update 
RUN apt-get install python3-catkin-tools -y 
RUN apt-get install libbenchmark-dev -y
RUN catkin config --extend /opt/ros/$ROS_DISTRO 
RUN catkin build 


