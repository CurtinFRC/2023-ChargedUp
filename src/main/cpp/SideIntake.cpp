#include "SideIntake.h"

using namespace frc;
using namespace wom;

SideIntake::SideIntake(SideIntakeConfig config) : _config(config) {}

void SideIntake::OnUpdate(units::second_t dt) {
  units::volt_t voltage = 0_V;

  switch (_state) {
    case SideIntakeState::kIdle:
      voltage = 0_V;
      break;

    case SideIntakeState::kIntaking:
      if (voltage == intakeVoltage) {
        voltage = intakeVoltage;
        _config.claspSolenoid->Set(frc::DoubleSolenoid::kForward);
      } else {
        voltage = intakeVoltage;
      }
      break;
    
    case SideIntakeState::kMovePiston:
      _config.deploySolenoid->Toggle();
      break;

    case SideIntakeState::kOuttaking:
      if (voltage == outtakeVoltage) {
        voltage = outtakeVoltage;
        _config.claspSolenoid->Set(frc::DoubleSolenoid::kReverse);
      } else {
        voltage = outtakeVoltage;
      }
      break;
  }
    _config.leftIntakeMotor->SetVoltage(voltage);
    _config.rightIntakeMotor->SetVoltage(voltage);
};

void SideIntake::SetIdle() {
  _state = SideIntakeState::kIdle;
}

void SideIntake::SetIntaking() {
  _state = SideIntakeState::kIntaking;
}

void SideIntake::SetPistons() {
  _state = SideIntakeState::kMovePiston;
}

void SideIntake::SetOuttaking() {
  _state = SideIntakeState::kOuttaking;
}

SideIntakeState SideIntake::GetState() const {
  return _state;
}