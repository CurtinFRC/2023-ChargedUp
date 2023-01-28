#pragma once

#include "RobotMap.h"

#include "behaviour/Behaviour.h"
#include "SideIntake.h"

class SideIntakeBehaviour : public behaviour::Behaviour {
 public:
  SideIntakeBehaviour(SideIntake *sideIntake, frc::XboxController &codriver);

  void OnStart() override;
  void OnTick(units::second_t dt) override;

 private:
  SideIntake *sideIntake;
  frc::XboxController &_codriver;
};