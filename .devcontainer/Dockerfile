FROM docker.io/ros:jazzy

ARG ROS_DISTRO
ENV ROS_DISTRO=$ROS_DISTRO
ARG USERNAME=okmr
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Install system packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    build-essential \
    cmake \
    git \
    git-lfs \
    pkg-config \
    python3-colcon-common-extensions \
    python3-rosdep \
    mesa-utils \
    x11-apps \
    libpcl-dev \
    libglm-dev \
    libsdl2-dev \
    libfreetype6-dev \
    libopengl-dev \
    python3-scipy \
    python3-transitions \
    && apt-get clean && rm -rf /var/lib/apt/lists/*


# Initialize rosdep
RUN rosdep init || true
RUN rosdep fix-permissions

# Switch to user context
USER $USERNAME
WORKDIR /home/$USERNAME
ENV SHELL=/bin/bash

# Setup environment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc