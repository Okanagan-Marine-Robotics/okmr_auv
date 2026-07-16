cmake_minimum_required(VERSION 3.8)
project(okmr_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(okmr_msgs REQUIRED)

add_executable(dead_reckoning
  src/dead_reckoning.cpp)

add_executable(relative_pose_target_server
  src/relative_pose_target_server.cpp)

add_executable(velocity_target_server
  src/velocity_target_server.cpp)

ament_target_dependencies(relative_pose_target_server rclcpp std_msgs okmr_msgs tf2 tf2_geometry_msgs geometry_msgs)

ament_target_dependencies(velocity_target_server rclcpp std_msgs okmr_msgs geometry_msgs)

ament_target_dependencies(dead_reckoning rclcpp std_msgs okmr_msgs tf2 tf2_geometry_msgs tf2_ros geometry_msgs sensor_msgs)

ament_python_install_package(${PROJECT_NAME})

install(TARGETS
    dead_reckoning
    relative_pose_target_server
    velocity_target_server
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
    ${PROJECT_NAME}/entry_point.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME navigator_action_server
)

if(BUILD_TESTING)
  # Integration tests
  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  function(add_ros_isolated_launch_test path)
    set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
    add_launch_test("${path}" RUNNER "${RUNNER}" ${ARGN})
  endfunction()
  add_ros_isolated_launch_test(test/test_move_relative_handler.py)
  add_ros_isolated_launch_test(test/test_navigation_integration.py)
endif()

ament_package()
