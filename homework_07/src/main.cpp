#include <cstdlib>
#include <iostream>

#include "mission_demo.hpp"

#ifndef HW7_DATA_DIR
#define HW7_DATA_DIR "."
#endif

#ifndef HW7_OUTPUT_DIR
#define HW7_OUTPUT_DIR "."
#endif

namespace {
const char *resolveDataDirectory()
{
  const char *overrideDir = std::getenv("HW7_DATA_DIR_OVERRIDE");
  if (overrideDir != nullptr && overrideDir[0] != '\0') {
    return overrideDir;
  }
  return HW7_DATA_DIR;
}

const char *resolveOutputDirectory()
{
  const char *overrideDir = std::getenv("HW7_OUTPUT_DIR_OVERRIDE");
  if (overrideDir != nullptr && overrideDir[0] != '\0') {
    return overrideDir;
  }
  return HW7_OUTPUT_DIR;
}
}  // namespace

int main()
{
  const char *dataDir = resolveDataDirectory();
  const char *outputDir = resolveOutputDirectory();

  const int status = runMissionDemo(dataDir, outputDir);
  if (status != 0) {
    std::cerr << "Mission demo failed" << std::endl;
  }
  return status;
}
