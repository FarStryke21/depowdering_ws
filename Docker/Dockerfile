# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Install NVIDIA support dependencies
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    ca-certificates \
    gnupg \
    lsb-release \ 
    gedit net-tools iputils-ping \
    python3-pip \
    python3-wstool \
    python3-catkin-tools \
    git \
    && rm -rf /var/lib/apt/lists/*

# Add the NVIDIA package repositories
RUN curl -s -L https://nvidia.github.io/libnvidia-container/gpgkey | apt-key add - && \
    distribution=$(. /etc/os-release;echo $ID$VERSION_ID) && \
    curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Install CA Certificates
RUN apt-get update && apt-get install -y ca-certificates
RUN update-ca-certificates

# Install Python Packages
RUN apt-get update && apt-get install -y python3-tk

# Update pip to >=20.3
RUN pip install -U pip>=20.3

RUN pip install --ignore-installed\
    open3d

# Clean up
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install ROS Packages
RUN apt-get install -y \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-realsense2-description \
    ros-${ROS_DISTRO}-catkin \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-rosserial \
    ros-${ROS_DISTRO}-rosserial-arduino \
    ros-${ROS_DISTRO}-ros-controllers

# Rosdep update
RUN rosdep update

WORKDIR /home/catkin_ws

# Create a symbolic link for python3 -> python
RUN ln -s /usr/bin/python3 /usr/bin/python

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

