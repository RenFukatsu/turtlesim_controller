FROM ros:melodic-ros-base

RUN apt update && \
    apt install -y ros-melodic-turtlesim python-catkin-tools

WORKDIR /root

RUN /bin/bash -c "mkdir -p catkin_ws/src"

RUN cd catkin_ws && /bin/bash -c "source /opt/ros/melodic/setup.bash; catkin build"

RUN cd /root && echo source /root/catkin_ws/devel/setup.bash >> .bashrc

ENV ROS_PACKAGE_PATH=/root/catkin_ws:$ROS_PACKAGE_PATH

ENV ROS_WORKSPACE=/root/catkin_ws

# RUN cd /root/catkin_ws/src && git clone https://github.com/RenFukatsu/turtlesim_controller.git

# RUN cd /root/catkin_ws && /bin/bash -c "catkin build"
