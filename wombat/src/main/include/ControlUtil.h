#pragma once

#include <cmath>

namespace wom {
  double deadzone(double val, double deadzone) {
    return std::fabs(val) > deadzone ? val : 0;
  }

  double spow2(double val) {
    return val*val*(val > 0 ? 1 : -1);
  }
}