#pragma once

#include "RobotMap.h"

#include "behaviour/Behaviour.h"
#include "PistonIntake.h"

class PistonIntakeBehaviour : public behaviour::Behaviour {
 public:
  PistonIntakeBehaviour(PistonIntake *pistonIntake, frc::XboxController &codriver);

  void OnStart() override;
  void OnTick(units::second_t dt) override;

 private:
  PistonIntake *pistonIntake;
  frc::XboxController &_codriver;
};