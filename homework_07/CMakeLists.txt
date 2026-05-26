cmake_minimum_required(VERSION 3.20)

project(homework_07_baseline CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(homework_07_baseline src/main.cpp)

target_include_directories(homework_07_baseline PRIVATE include)

target_compile_definitions(
    homework_07_baseline
    PRIVATE
        HW7_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
        HW7_OUTPUT_DIR="${CMAKE_CURRENT_BINARY_DIR}"
)

target_compile_options(homework_07_baseline PRIVATE -Wall -Wextra -pedantic)
