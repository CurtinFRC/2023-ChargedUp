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
        _config.solenoid1->Set(frc::DoubleSolenoid::kForward);
      } else {
        voltage = intakeVoltage;
      }
      break;
    
    case SideIntakeState::kMovePiston:
      _config.solenoid2->Toggle();
      break;

    case SideIntakeState::kOuttaking:
      if (voltage == outtakeVoltage) {
        voltage = outtakeVoltage;
        _config.solenoid1->Set(frc::DoubleSolenoid::kReverse);
      } else {
        voltage = outtakeVoltage;
      }
      break;
  }
    _config.motor1->SetVoltage(voltage);
    _config.motor2->SetVoltage(voltage);
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