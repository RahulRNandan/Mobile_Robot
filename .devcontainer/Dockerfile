FROM ubuntu:20.04

# Install dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Setup ROS 2 repository
RUN curl -sSL http://repo.ros2.org/repos.key | apt-key add - \
    && echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu focal main" | tee /etc/apt/sources.list.d/ros2-latest.list \
    && apt-get update

# Install ROS 2
RUN apt-get install -y \
    ros-foxy-desktop \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Setup environment
ENV ROS_DISTRO=foxy
RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Create and set working directory
WORKDIR /workspace
