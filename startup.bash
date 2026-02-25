read -p "THIS IS NOT INSTALL/SETUP.BASH. If you want that, press n RIGHT NOW!!!! (y/n): " response

case $response in
    [yY])
        echo "Commencing startup"

        # Source ROS to get $ROS_DISTRO
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
            ros-$ROS_DISTRO-librealsense2 \
            ros-$ROS_DISTRO-pcl-conversions \
            ros-$ROS_DISTRO-pcl-ros \
            ros-$ROS_DISTRO-image-transport \
            ros-$ROS_DISTRO-cv-bridge \
            ros-$ROS_DISTRO-diagnostic-updater \
            ros-$ROS_DISTRO-xacro \
            ros-$ROS_DISTRO-foxglove-bridge

        cd "$(dirname "$0")/ros2_ws"
        colcon build
        ;;
    *)
        exit
        ;;
esac
