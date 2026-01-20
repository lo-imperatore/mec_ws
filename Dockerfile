# Dockerfile
FROM osrf/ros:humble-desktop

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Basic tools + SSH client + Python tools
RUN apt-get update && apt-get install -y \
    git \
    ssh \
    nano \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    \
    # --- ros2_control (fixes "controller_manager not found") ---
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    \
    # Common robot description / URDF tools
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    \
    # Useful debugging & GUI tools (optional but nice)
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-rqt-topic \
    ros-humble-tf2-tools \
    \
    # PlotJuggler and plugins
    ros-humble-plotjuggler \
    ros-humble-plotjuggler-ros \
    \
    # For matplotlib
    python3-matplotlib \
    && rm -rf /var/lib/apt/lists/*
# Create a non-root user (optional but recommended)
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER $USERNAME
WORKDIR /home/$USERNAME

# Create colcon workspace
RUN mkdir -p /home/$USERNAME/colcon_ws/src
WORKDIR /home/$USERNAME/colcon_ws

# ROS 2 environment
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/colcon_ws/install/setup.bash" >> /home/$USERNAME/.bashrc

# Copy entrypoint script
COPY --chown=$USERNAME:$USERNAME entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
