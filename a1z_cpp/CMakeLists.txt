cmake_minimum_required(VERSION 3.16)
project(a1z_cpp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(CMAKE_BUILD_TYPE STREQUAL "Release")
        add_compile_options(-O2)
    endif()
endif()

# Dependencies
find_package(Threads REQUIRED)

# Optional: ROS2 for g4ros transport
find_package(rclcpp QUIET)
find_package(lemo_main_board QUIET)
if(rclcpp_FOUND AND lemo_main_board_FOUND)
    message(STATUS "ROS2 found: g4ros transport enabled")
    add_definitions(-DA1Z_HAS_ROS2)
    # ROS2 include directories
    include_directories(${rclcpp_INCLUDE_DIRS})
    include_directories(${lemo_main_board_INCLUDE_DIRS})
    # Get ROS2 library directories
    link_directories(${rclcpp_LIBRARY_DIRS})
    set(A1Z_ROS2_LIBS ${rclcpp_LIBRARIES} ${lemo_main_board_LIBRARIES})
else()
    message(STATUS "ROS2 not found: g4ros transport disabled")
endif()

# Optional: Pinocchio for gravity compensation
find_package(pinocchio QUIET)
if(pinocchio_FOUND)
    message(STATUS "Pinocchio found: gravity compensation enabled")
    add_definitions(-DA1Z_HAS_PINOCCHIO)
else()
    message(WARNING "Pinocchio not found: gravity compensation will be disabled")
endif()

# Optional: Eigen3 (required by Pinocchio)
find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 found")
else()
    message(WARNING "Eigen3 not found")
endif()

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen
)

# Source files
set(A1Z_SOURCES
    src/can_interface.cpp
    src/motor_a_driver.cpp
    src/motor_b_driver.cpp
    src/mixed_motor_chain.cpp
    src/gripper.cpp
    src/arm_robot.cpp
    src/transport_socketcan.cpp
    src/transport_g4ros.cpp
)

# Always include gravity_model (has stub when Pinocchio not available)
list(APPEND A1Z_SOURCES src/gravity_model.cpp)

# Library
add_library(a1z_cpp STATIC ${A1Z_SOURCES})

target_link_libraries(a1z_cpp
    PUBLIC
        Threads::Threads
        ${A1Z_ROS2_LIBS}
)

if(pinocchio_FOUND)
    target_link_libraries(a1z_cpp PUBLIC pinocchio::pinocchio)
endif()

if(Eigen3_FOUND)
    target_link_libraries(a1z_cpp PUBLIC Eigen3::Eigen)
endif()

# Tests (optional)
option(A1Z_BUILD_TESTS "Build tests" OFF)
if(A1Z_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# Examples (optional)
option(A1Z_BUILD_EXAMPLES "Build examples" OFF)
if(A1Z_BUILD_EXAMPLES)
    add_executable(motor_diag examples/motor_diag.cpp)
    target_link_libraries(motor_diag a1z_cpp)

    add_executable(gravity_comp examples/gravity_comp.cpp)
    target_link_libraries(gravity_comp a1z_cpp)
endif()

# Install
install(TARGETS a1z_cpp
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)
