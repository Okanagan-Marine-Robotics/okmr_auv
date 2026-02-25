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
            librealsense2-udev-rules \
            ros-jazzy-librealsense2 \
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
