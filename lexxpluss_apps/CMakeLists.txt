cmake_minimum_required(VERSION 3.13.1)

find_package(Zephyr)
project(lexxpluss_apps)

FILE(GLOB app_sources src/*.cpp)
FILE(GLOB ros_sources ../ros_msgs/*.cpp)
target_sources(app PRIVATE ${app_sources} ${ros_sources})
target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../ros_msgs)
