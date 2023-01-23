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
      voltage = intakeVoltage;
      _config.gearbox.transmission->SetVoltage(voltage);
      _config.solenoid1->Set(frc::DoubleSolenoid::kForward);
      break;
    
    case SideIntakeState::kMovePiston:
      _config.solenoid2->Toggle();
      break;

    case SideIntakeState::kOuttaking:
      voltage = outtakeVoltage;
      _config.solenoid1->Set(frc::DoubleSolenoid::kReverse);
      break;

  }
    _config.gearbox.transmission->SetVoltage(voltage);
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