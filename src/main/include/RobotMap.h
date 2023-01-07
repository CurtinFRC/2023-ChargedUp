#pragma once

#include "VoltageController.h"

#include <frc/XboxController.h>
#include <ctre/Phoenix.h>

struct RobotMap {
  struct Controllers {
    frc::XboxController driver{0};
    frc::XboxController codriver{1};
  };
  Controllers controllers;
};