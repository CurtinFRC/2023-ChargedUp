#pragma once
#include "VoltageController.h"

#include <frc/XboxController.h>
#include <ctre/Phoenix.h>

struct RobotMap {
  struct Controllers {
    frc::XboxController driver{0};
    frc::XboxController codriver{1};
  }; Controllers controllers;

  struct Vision {
    // units::meter_t offset_x = 0.3_m;
    // units::meter_t offset_y = 0.3_m;
  }; Vision vision;
};