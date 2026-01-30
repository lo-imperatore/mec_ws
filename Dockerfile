FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ----------------------------
# Core + ROS 2 packages (matched with the other Dockerfile)
# ----------------------------
RUN apt-get update && apt-get install -y \
    sudo \
    git \
    openssh-client \
    nano \
    tmux \
    xterm \
    pipx \
    chrony \
    iputils-ping \
    iproute2 \
    libeigen3-dev \
    python3-pip \
    python3-matplotlib \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    curl \
    lsb-release \
    gnupg \
    nautilus \
    \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ign-ros2-control \
    ros-humble-pinocchio \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-xacro \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-plotjuggler-ros \
    ros-humble-plotjuggler \
    ros-humble-rviz2 \
    ros-humble-rmw-cyclonedds-cpp \
    \
    ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------
# Gazebo (Ignition/Fortress) repository + install
# (matches the other Dockerfile approach)
# ----------------------------
RUN mkdir -p /usr/share/keyrings && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y ignition-fortress && \
    rm -rf /var/lib/apt/lists/*

# rosdep (must be root)
RUN rosdep init || true && rosdep update

# ----------------------------
# Non-root user
# ----------------------------
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -aG sudo,render,video $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

# pipx packages (installed as non-root, like the other Dockerfile)
RUN pipx ensurepath && \
    pipx install numpy

# Create colcon workspace
RUN mkdir -p /home/$USERNAME/colcon_ws/src
WORKDIR /home/$USERNAME/colcon_ws

# Source ROS env on shell startup + Gazebo plugin path (like the other Dockerfile)
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f /home/$USERNAME/colcon_ws/install/setup.bash ]; then source /home/$USERNAME/colcon_ws/install/setup.bash; fi" >> /home/$USERNAME/.bashrc && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib/" >> /home/$USERNAME/.bashrc

# Convenience: Eigen include symlink (matches the other Dockerfile)
USER root
RUN ln -sf /usr/include/eigen3/Eigen /usr/include/Eigen
USER $USERNAME

# Entrypoint
COPY --chown=$USERNAME:$USERNAME entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
