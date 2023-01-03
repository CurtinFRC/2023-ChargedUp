#pragma once

#include "behaviour/Behaviour.h"
#include "drivetrain/Drivetrain.h"

#include <frc/XboxController.h>

class DriveTeleopBehaviour : public behaviour::Behaviour {
 public: 
  DriveTeleopBehaviour(wom::Drivetrain *d, frc::XboxController *controller);
  void OnTick(units::second_t dt) override;
 private:
  wom::Drivetrain *_drivetrain;
  frc::XboxController *_controller;
};