#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <frc/simulation/EncoderSim.h>

namespace wom {
namespace sim {
  class SimCapableEncoder {
   public:
    virtual void SetEncoderTurns(units::turn_t turns) = 0;
    virtual void SetEncoderTurnVelocity(units::turns_per_second_t speed) = 0;
  };
}
}