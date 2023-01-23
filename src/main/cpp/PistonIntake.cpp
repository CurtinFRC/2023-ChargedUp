#include "PistonIntake.h"

using namespace frc;
using namespace wom;

PistonIntake::PistonIntake(PistonIntakeConfig config) : _config(config) {}

void PistonIntake::OnUpdate(units::second_t dt) {
  units::volt_t voltage = 0_V;


  switch (_state) {
    case PistonIntakeState::kIdle:
      voltage = 0_V;
      break;

    case PistonIntakeState::kIntaking:
      _config.solenoid->Set(frc::DoubleSolenoid::kForward);
      break;

    case PistonIntakeState::kOuttaking:
      _config.solenoid->Set(frc::DoubleSolenoid::kReverse);
      break;

  }
};

void PistonIntake::SetIdle() {
  _state = PistonIntakeState::kIdle;
}

void PistonIntake::SetIntaking() {
  _state = PistonIntakeState::kIntaking;
}


void PistonIntake::SetOuttaking() {
  _state = PistonIntakeState::kOuttaking;
}

PistonIntakeState PistonIntake::GetState() const {
  return _state;
}