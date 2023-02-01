#include "SideIntake.h"

using namespace frc;
using namespace wom;

SideIntake::SideIntake(SideIntakeConfig config) : _config(config) {}

void SideIntake::OnUpdate(units::second_t dt) {
  units::volt_t voltage = 0_V;

  switch (_state) {
  case SideIntakeState::kIdle:
    voltage = 0;
    break;

  case SideIntakeState::kStowed:
    _config.claspSolenoid->Set()
    break;

  case SideIntakeState::kIntakingWide:
    break;

  case SideIntakeState::kOuttakingWide:
    break;

  case SideIntakeState::kIntakingClosed:
    break;

  case SideIntakeState::kOutakingClosed:
    break;
  
  default:
    break;
  }


    _config.leftIntakeMotor->SetVoltage(voltage);
    _config.rightIntakeMotor->SetVoltage(voltage);
};

void SideIntake::SetIdle() {
  _state = SideIntakeState::kIdle;
}

void SideIntake::SetIntakingWide() {
  _state = SideIntakeState::kIntakingWide;
}

void SideIntake::SetOutakingWide() {
  _state = SideIntakeState::kOuttakingWide;
}

void SideIntake::SetIntakingClosed() {
  _state = SideIntakeState::kIntakingClosed;
}

void SideIntake::SetOutakingClosed() {
  _state = SideIntakeState::kOutakingClosed;
}

void SideIntake::SetStowed() {
  _state = SideIntakeState::kDeploy;
}
SideIntakeState SideIntake::GetState() const {
  return _state;
}