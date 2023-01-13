#pragma once

#include "gearbox.h"
#include "behaviour/Behaviour.h"
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include "Drivebase.h"
#include <frc/drive/MecanumDrive.h>

class ManualDrivebase : public behaviour::Behaviour{
 public:
  ManualDrivebase(MecanumDrivebase *mecanumDrivebase, frc::XboxController *driverController);

  void OnTick(units::second_t deltaTime);

 private:
  MecanumDrivebase *_mecanumDrivebase;
  frc::XboxController *_driverController;
  const double driverDeadzone = 0.05;
  const double turningDeadzone = 0.05;
  const units::meters_per_second_t maxMovementMagnitude = 0.5_mps;
};