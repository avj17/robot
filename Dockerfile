FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=burger
ENV GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib
ENV GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models


# Install development tools + simulation + navigation
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    nano \
    python3-colcon-common-extensions \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

# Create NEW docking workspace
RUN mkdir -p /root/docking_ws/src
WORKDIR /root/docking_ws

# Auto source ROS and workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/docking_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
