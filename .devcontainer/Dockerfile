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

# Install system packages and ROS dependencies
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
    ccache \
    ros-$ROS_DISTRO-librealsense2 \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-cv-bridge \
    ros-$ROS_DISTRO-diagnostic-updater \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-foxglove-bridge \
    && pip3 install --break-system-packages --ignore-installed onnx onnxruntime ultralytics \
    && pip3 uninstall -y setuptools --break-system-packages \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Build and Install Stonefish Globally
WORKDIR /tmp
RUN git clone https://github.com/patrykcieslak/Stonefish.git \
    && cd Stonefish \
    && git checkout 7d52673791834caa743907dde83a1949134ef2f0 \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && make -j$(nproc) \
    && make install \
    && mkdir -p /usr/local/share/Stonefish \
    && find .. -type d -name "fonts" -exec cp -r {} /usr/local/share/Stonefish/ \; \
    && cp -r ../Library/shaders /usr/local/share/Stonefish/ \
    && cd /tmp && rm -rf Stonefish

# Initialize rosdep
RUN rosdep init || true
RUN rosdep fix-permissions

# Switch to user context
USER $USERNAME
WORKDIR /home/$USERNAME
ENV SHELL=/bin/bash

# Setup environment
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc

# Enable ccache
ENV PATH="/usr/lib/ccache:$PATH"