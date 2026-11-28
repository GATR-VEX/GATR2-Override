# naviGATR host tests.
#
#   cmake -S lib/naviGATR -B build-navigatr       (from GATR2-Override)
#   cmake --build build-navigatr --parallel
#   ctest --test-dir build-navigatr --output-on-failure

cmake_minimum_required(VERSION 3.16)
project(navigatr CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# System gtest when present, fetched pinned copy otherwise.
find_package(GTest QUIET)
if(NOT GTest_FOUND)
    include(FetchContent)
    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

enable_testing()
include(GoogleTest)

add_library(navigatr STATIC
    src/route.cpp
    src/path.cpp
    src/profiles/profile.cpp
    src/pipeline.cpp
    src/profiles/trapezoid_profile.cpp
    src/followers/holonomic_follower.cpp
    src/drives/mecanum_drive.cpp
    src/driver/heading_hold.cpp
    src/driver/sticks.cpp
)
target_include_directories(navigatr PUBLIC
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../gatr2/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../locaGATR/include
)
if(NOT MSVC)
    target_compile_options(navigatr PRIVATE -Wall -Wextra -Werror)
endif()

# Test files.
set(NAVIGATR_TEST_SOURCES
    tests/route_gtest.cpp
    tests/path_gtest.cpp
    tests/pipeline_gtest.cpp
    tests/profiles/trapezoid_profile_gtest.cpp
    tests/followers/holonomic_follower_gtest.cpp
    tests/drives/mecanum_drive_gtest.cpp
    tests/driver/heading_hold_gtest.cpp
    tests/driver/sticks_gtest.cpp
)

add_executable(navigatr_tests ${NAVIGATR_TEST_SOURCES})
target_link_libraries(navigatr_tests PRIVATE navigatr GTest::gtest_main)
if(NOT MSVC)
    target_compile_options(navigatr_tests PRIVATE -Wall -Wextra -Werror)
endif()
gtest_discover_tests(navigatr_tests)
