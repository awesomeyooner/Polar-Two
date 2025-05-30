cmake_minimum_required(VERSION 3.8)
project(my_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(talker src/test/Publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/test/Subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(effort_translator src/util/EffortTranslator.cpp)
ament_target_dependencies(effort_translator rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(joystick_teleop src/util/Joystick.cpp)
ament_target_dependencies(joystick_teleop rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(gazebo_translator src/util/GazeboTranslator.cpp)
ament_target_dependencies(gazebo_translator rclcpp std_msgs geometry_msgs sensor_msgs)

add_executable(hardware_translator src/util/HardwareTranslator.cpp)
ament_target_dependencies(hardware_translator rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS
  talker
  listener
  effort_translator
  joystick_teleop
  gazebo_translator
  hardware_translator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  rviz
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

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
