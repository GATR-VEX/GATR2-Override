# locaGATR host tests. Needs GATR2-Electronics next to GATR2-Override.
#
#   cmake -S lib/locaGATR -B build-locagatr       (from GATR2-Override)
#   cmake --build build-locagatr --parallel
#   ctest --test-dir build-locagatr --output-on-failure

cmake_minimum_required(VERSION 3.16)
project(locagatr CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(GATR2_ELECTRONICS ${CMAKE_CURRENT_SOURCE_DIR}/../../../GATR2-Electronics
    CACHE PATH "GATR2-Electronics checkout")
if(NOT EXISTS ${GATR2_ELECTRONICS}/brain/locaGATR/src/link.cpp)
    message(FATAL_ERROR "GATR2-Electronics not found at ${GATR2_ELECTRONICS}")
endif()

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
find_package(Threads REQUIRED)

enable_testing()
include(GoogleTest)

add_library(locagatr STATIC
    src/types.cpp
    src/pipeline.cpp
    src/report.cpp
    src/sources/control_board_source.cpp
    src/processors/tracking_wheel_processor.cpp
    src/processors/wheel_pair_processor.cpp
    src/processors/drive_motor_processor.cpp
    src/estimators/odometry_estimator.cpp
    src/log/log_row.cpp
    ${GATR2_ELECTRONICS}/brain/locaGATR/src/link.cpp
    ${GATR2_ELECTRONICS}/pico/locaGATR/src/brainlink/wire.cpp
    ${GATR2_ELECTRONICS}/pico/locaGATR/src/records/encoder.cpp
)
target_include_directories(locagatr PUBLIC
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/../gatr2/include
    ${GATR2_ELECTRONICS}/brain/locaGATR/include
    ${GATR2_ELECTRONICS}/pico/locaGATR/include
)
if(NOT MSVC)
    target_compile_options(locagatr PRIVATE -Wall -Wextra -Werror)
endif()

# Test files.
set(LOCAGATR_TEST_SOURCES
    tests/pipeline_gtest.cpp
    tests/report_gtest.cpp
    tests/sources/control_board_source_gtest.cpp
    tests/processors/processors_gtest.cpp
    tests/estimators/odometry_estimator_gtest.cpp
    tests/slot_gtest.cpp
    tests/log/log_gtest.cpp
)

add_executable(locagatr_tests ${LOCAGATR_TEST_SOURCES})
target_link_libraries(locagatr_tests PRIVATE locagatr Threads::Threads GTest::gtest_main)
if(NOT MSVC)
    target_compile_options(locagatr_tests PRIVATE -Wall -Wextra -Werror)
endif()
gtest_discover_tests(locagatr_tests)
