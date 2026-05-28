cmake_minimum_required(VERSION 3.20)

project(homework_07_baseline CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(HW7_BASE_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data/base-circle")

add_executable(homework_07_baseline src/main.cpp src/file_config_loader.cpp
                                    src/json_target_provider.cpp src/analytical_solver.cpp)

target_include_directories(homework_07_baseline PRIVATE include)

target_compile_definitions(
    homework_07_baseline PRIVATE HW7_DATA_DIR="${HW7_BASE_DATA_DIR}"
                                 HW7_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}")

target_compile_options(homework_07_baseline PRIVATE -Wall -Wextra -pedantic)

if(NOT CMAKE_CROSSCOMPILING)
    include(FetchContent)
    include(GoogleTest)

    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(googletest)

    add_executable(
        homework_07_baseline_tests
        tests/baseline_tests.cpp
        tests/file_config_loader_tests.cpp
        tests/json_target_provider_tests.cpp
        tests/analytical_solver_tests.cpp
        src/file_config_loader.cpp
        src/json_target_provider.cpp
        src/analytical_solver.cpp)
    target_include_directories(homework_07_baseline_tests PRIVATE include)
    target_compile_definitions(
        homework_07_baseline_tests
        PRIVATE HW7_DATA_DIR="${HW7_BASE_DATA_DIR}"
                HW7_SCENARIOS_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/data"
                HW7_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-output"
                ENABLE_LOG=0
                ENABLE_DEBUG=0)
    target_compile_options(homework_07_baseline_tests PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(homework_07_baseline_tests PRIVATE GTest::gtest_main)

    gtest_discover_tests(homework_07_baseline_tests DISCOVERY_MODE PRE_TEST)
endif()
