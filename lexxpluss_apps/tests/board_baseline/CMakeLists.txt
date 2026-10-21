# Copyright (c) 2026, LexxPluss Inc.
# All rights reserved.
#
# Host harness comparing the board controller against the old baseline
# (commit 8c94ed6). Run:
#   cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure

cmake_minimum_required(VERSION 3.16)
project(board_baseline_tests CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra)

enable_testing()

set(OLD_COMMIT 8c94ed67dfa3e68e0c2aa007839b23dca4be1262)
set(OLD_SRC_DIR "${CMAKE_CURRENT_BINARY_DIR}/old_src")
get_filename_component(NEW_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../src" ABSOLUTE)

set(OLD_FILES
    board_controller.cpp board_controller.hpp common.hpp power_state.hpp
    adc_reader.hpp can_controller.hpp led_controller.hpp
    bmu_lipy041_decode.hpp bmu_lipy041_decode.cpp)

# Extract the old sources from git history (never edited)
set(OLD_OUTPUTS "")
foreach(file IN LISTS OLD_FILES)
    add_custom_command(
        OUTPUT "${OLD_SRC_DIR}/${file}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${OLD_SRC_DIR}"
        COMMAND git -C "${CMAKE_CURRENT_SOURCE_DIR}" show
                "${OLD_COMMIT}:lexxpluss_apps/src/${file}" > "${OLD_SRC_DIR}/${file}"
        VERBATIM)
    list(APPEND OLD_OUTPUTS "${OLD_SRC_DIR}/${file}")
endforeach()
add_custom_target(old_sources DEPENDS ${OLD_OUTPUTS})

set(COMMON_SOURCES
    test_runner.cpp
    stubs/zephyr_stubs.cpp
    stubs/lexxhard_stubs.cpp)

# Old baseline
add_executable(test_old ${COMMON_SOURCES} "${OLD_SRC_DIR}/bmu_lipy041_decode.cpp")
add_dependencies(test_old old_sources)
target_include_directories(test_old PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/stubs" "${OLD_SRC_DIR}")
target_compile_definitions(test_old PRIVATE "BC_SOURCE=\"${OLD_SRC_DIR}/board_controller.cpp\"")

# New sources
function(add_new_variant target build_name)
    add_executable(${target} ${COMMON_SOURCES}
        "${NEW_SRC_DIR}/bmu_lipy041_decode.cpp"
        "${NEW_SRC_DIR}/auto_charge_transitions.cpp")
    target_include_directories(${target} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/stubs" "${NEW_SRC_DIR}")
    target_compile_definitions(${target} PRIVATE
        BC_NEW_BUILD
        "BC_SOURCE=\"${NEW_SRC_DIR}/board_controller.cpp\""
        "BC_NEW_BUILD_NAME=\"${build_name}\""
        ${ARGN})
    set_property(TARGET ${target} APPEND PROPERTY LINK_DEPENDS "${NEW_SRC_DIR}/board_controller.cpp")
endfunction()

add_new_variant(test_new_standard "standard")
add_new_variant(test_new_push "push" ENABLE_PUSH_MODE=1)
add_new_variant(test_new_push_bypass "push+bypass"
    ENABLE_PUSH_MODE=1 BYPASS_SAFETY_LIDAR_FOR_AUTOCHARGE_TEST=1)

# The new-build runner exits 0 on SPEC_FAIL/DIFF, so the RESULT line is what is checked.
add_test(NAME baseline_old COMMAND test_old)
set_tests_properties(baseline_old PROPERTIES
    PASS_REGULAR_EXPRESSION "RESULT: 13 passed, 0 failed")

add_test(NAME new_standard COMMAND test_new_standard)
add_test(NAME new_push COMMAND test_new_push)
add_test(NAME new_push_bypass COMMAND test_new_push_bypass)
# Intended differences in push builds: S2, D3, D4, D7 (DIFF=4)
set_tests_properties(new_standard PROPERTIES
    PASS_REGULAR_EXPRESSION "RESULT: SAME=[0-9]+ DIFF=0 SPEC_PASS=[0-9]+ SPEC_FAIL=0")
set_tests_properties(new_push new_push_bypass PROPERTIES
    PASS_REGULAR_EXPRESSION "RESULT: SAME=[0-9]+ DIFF=4 SPEC_PASS=[0-9]+ SPEC_FAIL=0")
set_tests_properties(baseline_old new_standard new_push new_push_bypass PROPERTIES
    FAIL_REGULAR_EXPRESSION "HARNESS ERROR")
