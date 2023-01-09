#pragma once

#include <frc/RobotController.h>
#include <units/time.h>
namespace wom {
  template<typename T>
  T&& invert(T &&system) {
    system.SetInverted(true);
    return system;
  }

  units::second_t now();
}