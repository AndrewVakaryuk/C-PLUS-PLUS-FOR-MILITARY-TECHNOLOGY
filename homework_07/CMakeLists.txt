cmake_minimum_required(VERSION 3.20)

project(homework_07_baseline CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(HW7_BASE_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data/base-circle")

add_executable(
    homework_07_baseline
    src/main.cpp
    src/mission_demo.cpp
    src/mission_simulator.cpp
    src/simulation_json_writer.cpp
    src/ballistics.cpp
    src/drone_kinematics.cpp
    src/file_config_loader.cpp
    src/json_target_provider.cpp
    src/analytical_solver.cpp
    src/mission_processor.cpp
    src/factories.cpp)

target_include_directories(homework_07_baseline PRIVATE include)

target_compile_definitions(
    homework_07_baseline PRIVATE HW7_DATA_DIR="${HW7_BASE_DATA_DIR}"
                                 HW7_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}")

target_compile_options(homework_07_baseline PRIVATE -Wall -Wextra -pedantic)

if(NOT CMAKE_CROSSCOMPILING)
    include(GoogleTest)

    set(HW7_TEST_DEFINITIONS
        HW7_DATA_DIR="${HW7_BASE_DATA_DIR}" HW7_SCENARIOS_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/data"
        HW7_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}/test-output" ENABLE_LOG=0 ENABLE_DEBUG=0)
    set(HW7_REFACTORED_SOURCES
        src/mission_demo.cpp
        src/mission_simulator.cpp
        src/simulation_json_writer.cpp
        src/ballistics.cpp
        src/drone_kinematics.cpp
        src/file_config_loader.cpp
        src/json_target_provider.cpp
        src/analytical_solver.cpp
        src/mission_processor.cpp
        src/factories.cpp)

    add_executable(
        homework_07_baseline_tests
        tests/file_config_loader_tests.cpp tests/json_target_provider_tests.cpp
        tests/analytical_solver_tests.cpp tests/mission_processor_tests.cpp
        tests/mission_demo_tests.cpp ${HW7_REFACTORED_SOURCES})
    target_include_directories(homework_07_baseline_tests PRIVATE include)
    target_compile_definitions(homework_07_baseline_tests PRIVATE ${HW7_TEST_DEFINITIONS})
    target_compile_options(homework_07_baseline_tests PRIVATE -Wall -Wextra -pedantic)
    target_link_libraries(homework_07_baseline_tests PRIVATE GTest::gtest_main)
    gtest_discover_tests(homework_07_baseline_tests DISCOVERY_MODE PRE_TEST)
endif()
