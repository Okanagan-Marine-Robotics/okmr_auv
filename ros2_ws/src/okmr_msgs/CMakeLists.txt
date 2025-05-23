cmake_minimum_required(VERSION 3.8)
project(okmr_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/ImageWithPose.msg
    msg/GoalPose.msg
    msg/SensorReading.msg
    msg/MovementCommand.msg
    msg/MotorThrottle.msg
    msg/TempReading.msg
    msg/Status.msg
    msg/Dvl.msg
    srv/FindObject.srv
    srv/Vg2pc.srv
    srv/Matching.srv
    srv/Status.srv
    msg/VoxelGrid.msg
    msg/Classes.msg
  DEPENDENCIES sensor_msgs geometry_msgs std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
