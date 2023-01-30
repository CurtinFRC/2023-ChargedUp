#include "behaviour/SideIntakeBehaviour.h"

#include <iostream>

SideIntakeBehaviour::SideIntakeBehaviour(SideIntake *sideIntake, frc::XboxController &codriver): sideIntake(sideIntake), _codriver(codriver) {
  Controls(sideIntake);
}

void SideIntakeBehaviour::OnStart() {}

void SideIntakeBehaviour::OnTick(units::second_t dt) {
  if (_codriver.GetAButton()) {
    sideIntake->SetIntaking();
  } else if (_codriver.GetBButton()) {
    sideIntake->SetOuttaking();
  } else if (_codriver.GetXButton())  {
    sideIntake->SetPistons();
  } else {
    sideIntake->SetIdle();
  }
}