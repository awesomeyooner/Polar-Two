cmake_minimum_required(VERSION 3.8)
project(camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  backward_ros
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  camera_driver_srvs
  generate_parameter_library
  CURL
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
  include
  ${OpenCV_INCLUDE_DIRS}
)

# uncomment the following section in order to fill in
# further dependencies manually.
# <dependency> REQUIRED)

add_executable(camera_driver_node src/Main.cpp)
ament_target_dependencies(camera_driver_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

generate_parameter_library(
  ${PROJECT_NAME}_parameters
  src/parameters.yaml
)

target_link_libraries(camera_driver_node ${OpenCV_LIBS} CURL::libcurl ${PROJECT_NAME}_parameters)

install(TARGETS
  camera_driver_node
  ${PROJECT_NAME}_parameters
  EXPORT ${PROJECT_NAME}Targets
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

install(DIRECTORY
  launch
  config
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
