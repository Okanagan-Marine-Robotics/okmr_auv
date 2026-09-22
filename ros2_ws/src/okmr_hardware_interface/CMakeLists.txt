cmake_minimum_required(VERSION 3.8)
project(okmr_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(okmr_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(include)

add_executable(thrust_to_pwm_node
  src/thrust_to_pwm.cpp
)

add_executable(fire_torpedo_server
  src/fire_to_actuator.cpp
)

ament_target_dependencies(thrust_to_pwm_node
  rclcpp
  okmr_msgs
)

ament_target_dependencies(fire_torpedo_server
  okmr_msgs
  rclcpp
  rclcpp_action
)

#Include local headers for now should be removed or altered once resolved
target_include_directories(fire_torpedo_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install C++ executables
install(TARGETS
  thrust_to_pwm_node
  fire_torpedo_server
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    ${PROJECT_NAME}/dvl_driver.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME dvl_driver
)

install(PROGRAMS
    ${PROJECT_NAME}/esp32_bridge.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME esp32_bridge
)

install(PROGRAMS
    ${PROJECT_NAME}/temp_sensor.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME temp_sensor
)

install(PROGRAMS
    ${PROJECT_NAME}/clutch_mega_driver.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME clutch_mega_driver
)


install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY tables/
  DESTINATION share/${PROJECT_NAME}/tables
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

ament_export_dependencies(rclcpp okmr_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
