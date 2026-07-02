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
    msg/BatteryVoltage.msg
    msg/BoundingBox.msg
    msg/ImageWithPose.msg
    msg/GoalPose.msg
    msg/GoalVelocity.msg
    msg/GoalAccel.msg
    msg/GoalWrench.msg
    msg/GoalThrottle.msg
    msg/ControlMode.msg
    msg/SensorReading.msg
    msg/MovementCommand.msg
    msg/MotorThrottle.msg
    msg/MotorThrust.msg
    msg/MaskOffset.msg
    msg/TempReading.msg
    msg/Status.msg
    msg/Dvl.msg
    msg/RelativePose.msg
    msg/MissionCommand.msg
    msg/SemanticDepth.msg
    msg/ServoCommand.msg
    msg/ActuatorCommand.msg
    msg/DetectedObject.msg
    msg/EnvironmentData.msg
    srv/FindObject.srv
    srv/Vg2pc.srv
    srv/Matching.srv
    srv/Status.srv
    srv/GetPoseTwistAccel.srv
    srv/SetDeadReckoningEnabled.srv
    srv/EnableThrustAllocation.srv
    srv/ClearPose.srv
    srv/DistanceFromGoal.srv
    srv/ChangeModel.srv
    srv/SetInferenceCamera.srv
    msg/VoxelGrid.msg
    msg/Classes.msg
    action/Movement.action
    action/Arm.action
    srv/ArmTarget.srv
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
