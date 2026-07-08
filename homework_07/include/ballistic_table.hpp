#pragma once

#include <cstddef>
#include <string>
#include <vector>

class BallisticTable {
public:
  struct Result {
    double t;
    double hDist;
  };

  bool load(const std::string &path);
  Result lookup(double z0, double v0, double m, double d, double l) const;
  bool isLoaded() const;
  bool supports(double z0, double v0, double m, double d, double l) const;

private:
  struct Interp {
    int lo;
    double frac;
  };

  std::size_t index(int iz, int iv, int im, int id, int il) const;
  const Result &at(int iz, int iv, int im, int id, int il) const;
  static Result lerp(const Result &a, const Result &b, double t);
  static Interp findInterp(double value, const std::vector<double> &axis);

  std::vector<double> axisZ0_;
  std::vector<double> axisV0_;
  std::vector<double> axisM_;
  std::vector<double> axisD_;
  std::vector<double> axisL_;
  std::vector<Result> data_;
};
