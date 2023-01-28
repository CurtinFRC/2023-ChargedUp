#include "behaviour/SideIntakeBehaviour.h"

#include <iostream>

SideIntakeBehaviour::SideIntakeBehaviour(SideIntake *sideIntake, frc::XboxController &codriver): sideIntake(sideIntake), _codriver(codriver) {
  Controls(sideIntake);
}

void SideIntakeBehaviour::OnStart() {}

void SideIntakeBehaviour::OnTick(units::second_t dt) {
  if (_codriver.GetAButton()) {
    sideIntake->SetIntaking();
  } if (_codriver.GetBButton()) {
    sideIntake->SetOuttaking();
  } if (_codriver.GetXButton())  {
    sideIntake->SetMovePiston();
  } if (_codriver.GetYButton()){
    sideIntake->SetClaspPiston();
  }

}