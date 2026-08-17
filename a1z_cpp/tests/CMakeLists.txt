# Tests for a1z_cpp

# Basic functionality test (no hardware required)
add_executable(test_basic test_basic.cpp)
target_link_libraries(test_basic a1z_cpp)

# Add test
add_test(NAME basic COMMAND test_basic)
