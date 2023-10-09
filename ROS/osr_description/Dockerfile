FROM ros:foxy

# install dependencies
RUN apt-get update \
 && apt-get -y install \
    curl \
    gnupg2 \
    lsb-release \
    wget \
    git \
    nano \
    python3-dev \
    python3-pip \
    locales \
    pkg-config \
    python3-yaml \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip

# set up gazebo sources
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# install some ROS packages
RUN apt-get update && apt-get -y upgrade
RUN apt-get -y install \
    ros-foxy-rqt \
    ros-foxy-rviz2 \
    qt5-default

RUN echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc

# # nvidia-container-runtime
# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

WORKDIR /osr_ws/src/osr_description