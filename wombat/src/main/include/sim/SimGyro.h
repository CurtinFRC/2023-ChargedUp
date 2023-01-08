#pragma once

#include <units/angle.h>

namespace wom {
namespace sim {
  class SimCapableGyro {
   public:
    virtual void SetAngle(units::radian_t angle) = 0;
  };
}
}