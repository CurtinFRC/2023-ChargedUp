#pragma once 

// #include "RobotMap.h"
#include "behaviour/Behaviour.h"
#include "Vision.h"
#include "drivetrain/SwerveDrive.h"
#include <frc/XboxController.h>

class VisionBehaviour : public behaviour::Behaviour {
 public:
  // VisionBehaviour(Vision *vision, frc::XboxController *codriver);
  VisionBehaviour(Vision *vision, wom::SwerveDrive *swerveDrivebase, frc::XboxController *codriver);

  void OnTick(units::second_t dt) override;

 private: 
  wom::SwerveDrive *_swerveDrivebase;
  Vision *_vision;
  frc::XboxController *_codriver;
};