cmake_minimum_required(VERSION 3.8)
project(uros_helper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Create list of dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_msgs
  ament_cmake
)

# Find each dependency
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Install Extra Folders
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # target_link_libraries(${PROJECT_NAME}_test my_exec)
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
