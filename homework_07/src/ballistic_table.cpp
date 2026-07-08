#include "ballistic_table.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace {
bool readAxis(std::istream &input, std::vector<double> &axis)
{
  for (double &value : axis) {
    input >> value;
    if (!input) {
      return false;
    }
  }
  return true;
}
}  // namespace

bool BallisticTable::load(const std::string &path)
{
  axisZ0_.clear();
  axisV0_.clear();
  axisM_.clear();
  axisD_.clear();
  axisL_.clear();
  data_.clear();

  std::ifstream file(path, std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Error: could not open ballistic table file: " << path << std::endl;
    return false;
  }

  // Normalize CRLF so Windows-exported tables parse cleanly.
  std::ostringstream normalized;
  normalized << file.rdbuf();
  std::string content = normalized.str();
  content.erase(std::remove(content.begin(), content.end(), '\r'), content.end());

  std::istringstream input(content);
  int nZ = 0;
  int nV = 0;
  int nM = 0;
  int nD = 0;
  int nL = 0;
  input >> nZ >> nV >> nM >> nD >> nL;
  if (!input || nZ < 2 || nV < 2 || nM < 2 || nD < 2 || nL < 2) {
    std::cerr << "Error: invalid ballistic table header in " << path << std::endl;
    return false;
  }

  axisZ0_.resize(static_cast<std::size_t>(nZ));
  axisV0_.resize(static_cast<std::size_t>(nV));
  axisM_.resize(static_cast<std::size_t>(nM));
  axisD_.resize(static_cast<std::size_t>(nD));
  axisL_.resize(static_cast<std::size_t>(nL));

  if (!readAxis(input, axisZ0_) || !readAxis(input, axisV0_) || !readAxis(input, axisM_) || !readAxis(input, axisD_) ||
      !readAxis(input, axisL_)) {
    std::cerr << "Error: invalid ballistic table axes in " << path << std::endl;
    axisZ0_.clear();
    axisV0_.clear();
    axisM_.clear();
    axisD_.clear();
    axisL_.clear();
    return false;
  }

  const std::size_t total =
    static_cast<std::size_t>(nZ) * static_cast<std::size_t>(nV) * static_cast<std::size_t>(nM) * static_cast<std::size_t>(nD) *
    static_cast<std::size_t>(nL);
  data_.resize(total);
  for (Result &entry : data_) {
    input >> entry.t >> entry.hDist;
    if (!input) {
      std::cerr << "Error: truncated ballistic table data in " << path << std::endl;
      data_.clear();
      return false;
    }
  }

  return true;
}

BallisticTable::Result BallisticTable::lookup(double z0, double v0, double m, double d, double l) const
{
  const Interp iz = findInterp(z0, axisZ0_);
  const Interp iv = findInterp(v0, axisV0_);
  const Interp im = findInterp(m, axisM_);
  const Interp id = findInterp(d, axisD_);
  const Interp il = findInterp(l, axisL_);

  Result values[16];
  for (int a = 0; a < 2; a++) {
    for (int b = 0; b < 2; b++) {
      for (int c = 0; c < 2; c++) {
        for (int e = 0; e < 2; e++) {
          const Result &lo = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
          const Result &hi = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
          values[a * 8 + b * 4 + c * 2 + e] = lerp(lo, hi, il.frac);
        }
      }
    }
  }

  Result dReduced[8];
  for (int a = 0; a < 2; a++) {
    for (int b = 0; b < 2; b++) {
      for (int c = 0; c < 2; c++) {
        dReduced[a * 4 + b * 2 + c] = lerp(values[a * 8 + b * 4 + c * 2], values[a * 8 + b * 4 + c * 2 + 1], id.frac);
      }
    }
  }

  Result mReduced[4];
  for (int a = 0; a < 2; a++) {
    for (int b = 0; b < 2; b++) {
      mReduced[a * 2 + b] = lerp(dReduced[a * 4 + b * 2], dReduced[a * 4 + b * 2 + 1], im.frac);
    }
  }

  Result vReduced[2];
  for (int a = 0; a < 2; a++) {
    vReduced[a] = lerp(mReduced[a * 2], mReduced[a * 2 + 1], iv.frac);
  }
  return lerp(vReduced[0], vReduced[1], iz.frac);
}

bool BallisticTable::isLoaded() const
{
  return !data_.empty();
}

bool BallisticTable::supports(double z0, double v0, double m, double d, double l) const
{
  if (!isLoaded()) {
    return false;
  }

  // Ammo/config values arrive as float and may not match printed double axes exactly.
  constexpr double kEps = 1e-5;
  auto inRange = [](double value, const std::vector<double> &axis) {
    return value >= axis.front() - kEps && value <= axis.back() + kEps;
  };

  return inRange(z0, axisZ0_) && inRange(v0, axisV0_) && inRange(m, axisM_) && inRange(d, axisD_) && inRange(l, axisL_);
}

std::size_t BallisticTable::index(int iz, int iv, int im, int id, int il) const
{
  return ((((static_cast<std::size_t>(iz) * axisV0_.size() + static_cast<std::size_t>(iv)) * axisM_.size()) +
           static_cast<std::size_t>(im)) *
            axisD_.size() +
          static_cast<std::size_t>(id)) *
           axisL_.size() +
         static_cast<std::size_t>(il);
}

const BallisticTable::Result &BallisticTable::at(int iz, int iv, int im, int id, int il) const
{
  return data_[index(iz, iv, im, id, il)];
}

BallisticTable::Result BallisticTable::lerp(const Result &a, const Result &b, double t)
{
  return Result{a.t + (b.t - a.t) * t, a.hDist + (b.hDist - a.hDist) * t};
}

BallisticTable::Interp BallisticTable::findInterp(double value, const std::vector<double> &axis)
{
  if (value <= axis.front()) {
    return Interp{0, 0.0};
  }
  if (value >= axis.back()) {
    return Interp{static_cast<int>(axis.size()) - 2, 1.0};
  }

  const auto it = std::lower_bound(axis.begin(), axis.end(), value);
  int low = static_cast<int>(it - axis.begin()) - 1;
  if (low < 0) {
    low = 0;
  }
  const double denom = axis[static_cast<std::size_t>(low + 1)] - axis[static_cast<std::size_t>(low)];
  const double frac = (denom == 0.0) ? 0.0 : (value - axis[static_cast<std::size_t>(low)]) / denom;
  return Interp{low, frac};
}
