cmake_minimum_required(VERSION 3.16)
project(push_mode_tests)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../src")

# Standard mode: no ENABLE_PUSH_MODE
add_executable(test_normal_transitions_standard
    test_normal_transitions.cpp
)
target_include_directories(test_normal_transitions_standard PRIVATE ${SRC_DIR})
target_link_libraries(test_normal_transitions_standard gtest gtest_main)

# Push mode: ENABLE_PUSH_MODE=1
add_executable(test_normal_transitions_push_mode
    test_normal_transitions.cpp
)
target_include_directories(test_normal_transitions_push_mode PRIVATE ${SRC_DIR})
target_compile_definitions(test_normal_transitions_push_mode PRIVATE ENABLE_PUSH_MODE=1)
target_link_libraries(test_normal_transitions_push_mode gtest gtest_main)

enable_testing()
add_test(NAME standard_mode COMMAND test_normal_transitions_standard)
add_test(NAME push_mode     COMMAND test_normal_transitions_push_mode)
