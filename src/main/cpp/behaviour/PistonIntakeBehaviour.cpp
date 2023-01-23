#include "behaviour/PistonIntakeBehaviour.h"

#include <iostream>

PistonIntakeBehaviour::PistonIntakeBehaviour(PistonIntake *pistonIntake, frc::XboxController &codriver): pistonIntake(pistonIntake), _codriver(codriver) {
  Controls(pistonIntake);
}

void PistonIntakeBehaviour::OnStart() {}

void PistonIntakeBehaviour::OnTick(units::second_t dt) {
  if (_codriver.GetAButton()) {
    pistonIntake->SetIntaking();
  } if (_codriver.GetBButton()) {
    pistonIntake->SetOuttaking();
  }
}