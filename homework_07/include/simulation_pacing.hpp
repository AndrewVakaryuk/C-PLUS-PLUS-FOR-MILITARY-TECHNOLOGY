#pragma once

#include <chrono>
#include <thread>

inline void paceSimulation(double simDt, double timeScale)
{
  if (timeScale <= 0.0) {
    return;
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(simDt / timeScale));
}
