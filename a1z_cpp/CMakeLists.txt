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
)

# Always include gravity_model (has stub when Pinocchio not available)
list(APPEND A1Z_SOURCES src/gravity_model.cpp)

# Library
add_library(a1z_cpp STATIC ${A1Z_SOURCES})

target_link_libraries(a1z_cpp
    PUBLIC
        Threads::Threads
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

# Install
install(TARGETS a1z_cpp
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)
