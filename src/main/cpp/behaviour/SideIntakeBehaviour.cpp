#include "behaviour/SideIntakeBehaviour.h"

#include <iostream>

SideIntakeBehaviour::SideIntakeBehaviour(SideIntake *sideIntake, frc::XboxController &codriver): sideIntake(sideIntake), _codriver(codriver) {
  Controls(sideIntake);
}

void SideIntakeBehaviour::OnStart() {}

void SideIntakeBehaviour::OnTick(units::second_t dt) {
  if (_codriver.GetLeftBumper()) {
    sideIntake->SetIntaking();
  } else if (_codriver.GetRightBumper()) {
    sideIntake->SetOuttaking();
  } else if (_codriver.GetLeftTriggerAxis()>0.15)  {
    sideIntake->SetPistons();
  } else {
    sideIntake->SetIdle();
  }

  if ()
}