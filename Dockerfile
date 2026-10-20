# osr_gazebo depends on Gazebo Classic, which is only supported on ROS 2 Humble
ARG ROS_VERSION=humble
FROM osrf/ros:${ROS_VERSION}-desktop

SHELL ["/bin/bash", "-c"]

# need to redefine argument since we started fresh with FROM
ARG ROS_VERSION
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    nano \
    python3-rosdep \
    python3-pip \
    python3-venv

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME
# explicitly set HOME variable since otherwise ${HOME} only applies to RUN, CMD, and ENTRYPOINT commands and would otherwise evaluate to '/'
ENV HOME=/home/${USERNAME}

COPY --chown=$USERNAME:$USERNAME ./ ${HOME}/osr_ws/src/osr-rover-code

WORKDIR ${HOME}/osr_ws
# create a python virtual environment for the rover's python dependencies and
# keep colcon from trying to build it
RUN python3 -m venv venv --system-site-packages \
    && touch ./venv/COLCON_IGNORE
# Gazebo Classic simulation packages (osr_gazebo)
RUN sudo apt-get update && sudo apt-get install --no-install-recommends -y \
    python3-colcon-common-extensions \
    ros-${ROS_VERSION}-rviz2 \
    ros-${ROS_VERSION}-controller-manager \
    ros-${ROS_VERSION}-robot-state-publisher \
    ros-${ROS_VERSION}-joint-state-publisher \
    ros-${ROS_VERSION}-joint-state-publisher-gui \
    ros-${ROS_VERSION}-gazebo-ros-pkgs \
    ros-${ROS_VERSION}-trajectory-msgs \
    ros-${ROS_VERSION}-velocity-controllers \
    ros-${ROS_VERSION}-joint-trajectory-controller \
    ros-${ROS_VERSION}-gazebo-ros2-control-demos
RUN rosdep update
RUN sudo apt-get update && source ./venv/bin/activate \
    && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_VERSION} -y
RUN source ./venv/bin/activate \
    && python3 -m pip install adafruit-circuitpython-servokit ina260 RPi.GPIO smbus
# osr_gazebo ships with a COLCON_IGNORE since it's optional (see its README);
# remove it so the simulation is built and ready to run out of the box
RUN rm -f src/osr-rover-code/ROS/osr_gazebo/COLCON_IGNORE
RUN source /opt/ros/${ROS_VERSION}/setup.bash && source ./venv/bin/activate \
    && python3 -m colcon build --symlink-install
# create the modifiable config files that overwrite the default parameters
RUN touch src/osr-rover-code/ROS/osr_bringup/config/osr_params_mod.yaml \
    src/osr-rover-code/ROS/osr_bringup/config/roboclaw_params_mod.yaml
RUN echo "source ${HOME}/osr_ws/venv/bin/activate" >> ~/.bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source ${HOME}/osr_ws/install/setup.bash" >> ~/.bashrc
