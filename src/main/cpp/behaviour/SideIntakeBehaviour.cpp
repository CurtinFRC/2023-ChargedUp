#include "behaviour/SideIntakeBehaviour.h"

#include <iostream>

SideIntakeBehaviour::SideIntakeBehaviour(SideIntake *sideIntake, frc::XboxController &codriver): sideIntake(sideIntake), _codriver(codriver) {
  Controls(sideIntake);
}

void SideIntakeBehaviour::OnStart() {}

void SideIntakeBehaviour::OnTick(units::second_t dt) {
  if (_codriver.GetBButtonPressed()) {
    if (_intakeReleased) {
      _intakeReleased = false;
    } else {
      _intakeReleased = true;
    }
  }

  if (_codriver.GetXButtonPressed()) {
    if (_intakeGrapper) {
      _intakeGrapper = false;
    } else {
      _intakeGrapper = true;
    }
  }

  if (_intakeReleased) {
    sideIntake->SetStow();
  } else {
    sideIntake->SetDeploy();
  }

  if (_intakeGrapper) {
    sideIntake->SetClose();
  } else {
    sideIntake->SetOpen();
  }

  // std::cout << sideIntake->GetState() << std::endl;

  if (wom::deadzone(_codriver.GetRightTriggerAxis())) {
    sideIntake->SetVoltage(_codriver.GetRightTriggerAxis() * 10_V);
  } else if (wom::deadzone(_codriver.GetLeftTriggerAxis())) {
    sideIntake->SetVoltage(_codriver.GetLeftTriggerAxis() * -10_V);
  } else {
    sideIntake->SetVoltage(0_V);
  }
}