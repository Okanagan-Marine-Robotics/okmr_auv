cmake_minimum_required(VERSION 3.8)
project(dropper_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(okmr_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(rclcpp_components REQUIRED) # currently unused

# allows the action server to compile
add_executable(dropper_action_server
  src/dropper_action.cpp)
target_include_directories(dropper_action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dropper_action_server
  "okmr_msgs"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
install(TARGETS
  dropper_action_server
  DESTINATION lib/${PROJECT_NAME})


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

ament_package()
