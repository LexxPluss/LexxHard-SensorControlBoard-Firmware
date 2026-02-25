cmake_minimum_required(VERSION 3.14)
project(pgood_debouncer_tests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL      https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
    URL_HASH SHA256=8ad598c73ad796e0d8280b082cebd82a630d73e73cd3c70057938a6501bba5d7
)
# Prevent GoogleTest from overriding compiler/linker options on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(pgood_debouncer_tests
    test_pgood_debouncer.cpp
)

target_include_directories(pgood_debouncer_tests PRIVATE
    ../../lexxpluss_apps/src
)

target_link_libraries(pgood_debouncer_tests
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(pgood_debouncer_tests)
