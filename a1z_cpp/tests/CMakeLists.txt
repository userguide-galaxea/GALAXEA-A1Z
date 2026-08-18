# Tests for a1z_cpp

# Basic functionality test (no hardware required)
add_executable(test_basic test_basic.cpp)
target_link_libraries(test_basic a1z_cpp)
# Keep assert() live even in Release builds (NDEBUG would no-op the checks).
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(test_basic PRIVATE -UNDEBUG)
endif()

# Add test
add_test(NAME basic COMMAND test_basic)
