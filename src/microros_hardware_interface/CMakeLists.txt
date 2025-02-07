cmake_minimum_required(VERSION 3.16)
project(microros_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(
  nlohmann_json 3.2.0 REQUIRED
  CONFIG
    PATHS "/home/dev/anaconda3/share/cmake/nlohmann_json"
    )


## COMPILE
add_library(
  microros_hardware_interface
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(microros_hardware_interface PUBLIC cxx_std_17)
target_include_directories(microros_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/microros_hardware_interface>
)
ament_target_dependencies(
  microros_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#include(/home/dev/cget/cget/cget.cmake)

list(APPEND CMAKE_MODULE_PATH ${CGET_PREFIX}/share/cmake)

find_package(nlohmann_json REQUIRED)

target_link_libraries(microros_hardware_interface PUBLIC serial nlohmann_json::nlohmann_json)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_example_11_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface microros_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY hardware/include
  DESTINATION include/microros_hardware_interface
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/microros_hardware_interface
)
install(
  DIRECTORY bringup/launch bringup/config bringup
  DESTINATION share/microros_hardware_interface
)

install(TARGETS microros_hardware_interface
  EXPORT export_microros_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(microros_hardware_interface_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_microros_hardware_interface_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_microros_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
