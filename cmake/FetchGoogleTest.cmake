# Fetch GoogleTest once for the whole tree (homework_06, homework_07, ...).
# Skip on cross-compiles where native gtest binaries cannot run on the host.

if(CMAKE_CROSSCOMPILING OR TARGET GTest::gtest_main)
  return()
endif()

include(FetchContent)

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)

# MSVC: keep CRT settings consistent with the parent project.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_MakeAvailable(googletest)

list(APPEND CMAKE_MODULE_PATH ${googletest_SOURCE_DIR}/googletest/cmake)
