cmake_minimum_required(VERSION 3.12)

# CHANGE ME
project( project LANGUAGES C CXX ) # C CXX allows for both C and C++ to be compiled

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Variables for what to call the main and text executables
set(MAIN_EXEC Main)
set(TEST_EXEC Test)

# Create list of dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  # Put your dependencies here
)

# List of all .cpp files
set(SRC_FILES
    # src/my_file.cpp
    # src/main.cpp
)

# Find each dependency
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Create the main executable
add_executable( ${MAIN_EXEC} src/main.cpp ${SRC_FILES})

target_include_directories(${MAIN_EXEC}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # If you have a /lib folder then uncomment these
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
        $<INSTALL_INTERFACE:lib>
)

# Add libraries using
# add_subdirectory(lib/MyLib)
# target_link_libraries( ${MAIN_EXEC} MyLib )

add_subdirectory(lib/plib)
target_link_libraries( ${MAIN_EXEC} plib )

add_subdirectory(lib/PlotLib)
target_link_libraries( ${MAIN_EXEC} PlotLib )

add_subdirectory(lib/CommiFaceLib)
target_link_libraries( ${MAIN_EXEC} CommiFaceLib )

target_link_libraries( ${MAIN_EXEC} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# === TESTING ===

# GTest
find_package(GTest CONFIG REQUIRED)
include(GoogleTest)

enable_testing()

# Create the test executable
add_executable( ${TEST_EXEC} test/test.cpp)

# Link the gtest library
target_link_libraries(${TEST_EXEC} GTest::gtest_main)

# Add the include dirs
target_include_directories(${TEST_EXEC}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        # If you have a /lib folder then uncomment these
        # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
        # $<INSTALL_INTERFACE:lib>
)

# For tests to be discovered automatically
gtest_discover_tests(${TEST_EXEC})

# For tests to be manually created
# add_test(NAME my_test_case COMMAND my_test)
