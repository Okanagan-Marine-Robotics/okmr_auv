read -p "THIS IS NOT INSTALL/SETUP.BASH. If you want that, press n RIGHT NOW!!!! (y/n): " response

case $response in
    [yY])
        echo "Commencing startup"

        if [ ! -f /opt/ros/jazzy/setup.bash ]; then
            echo "ERROR: ROS Jazzy is not installed."
            echo "Install it first: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html"
            echo "You got this."
            exit 1
        fi

        source /opt/ros/jazzy/setup.bash

        # Add Intel RealSense apt repo
        sudo mkdir -p /etc/apt/keyrings
        curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
        echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list
        sudo apt update

        sudo apt install -y \
            python3-colcon-common-extensions \
            python3-rosdep \
            libpcl-dev \
            libglm-dev \
            libsdl2-dev \
            libfreetype-dev \
            libopengl-dev \
            python3-scipy \
            python3-transitions \
            librealsense2 \
            librealsense2-dev \
            librealsense2-gl \
            librealsense2-udev-rules \
            librealsense2-utils \
            ros-jazzy-pcl-conversions \
            ros-jazzy-pcl-ros \
            ros-jazzy-image-transport \
            ros-jazzy-cv-bridge \
            ros-jazzy-diagnostic-updater \
            ros-jazzy-xacro \
            ros-jazzy-foxglove-bridge

        cd "$(dirname "$0")/ros2_ws"
        colcon build
        ;;
    *)
        exit
        ;;
esac
