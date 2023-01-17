#pragma once

#include "SwerveMod.h"
#include "behaviour/Behaviour.h"
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

class SwerveSingleModManual : public behaviour::Behaviour{
 public:
  SwerveSingleModManual(SwerveModuleTest *swerveModuleTest, frc::XboxController *driverController);

  void OnUpdate(units::second_t deltaTime);

 private:
  SwerveModuleTest *_swerveTestModule;
  frc::XboxController *_driverController;
  const double driverDeadzone = 0.05;
  const double turningDeadzone = 0.1;
  const units::meters_per_second_t maxMovementMagnitude = 16_ft / 1_s;
};