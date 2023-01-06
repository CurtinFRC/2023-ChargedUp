#include "Intake.h"

using namespace frc;
using namespace wom;

Intake::Intake(IntakeConfig config) : _config(config) { }

void Intake::OnUpdate(units::second_t dt) {
  units::volt_t voltage = 0_V;

  /* Depending on the state, we set a different voltage */
  switch (_state) {
  case IntakeState::kIdle:
    voltage = 0_V;
    break;
  case IntakeState::kIntaking:
    voltage = _config.intakeVoltage;
    /* If we have a gear, move to kFull */
    if (_config.gearPresenceSensor->Get())
      _state = IntakeState::kFull;
    break;
  case IntakeState::kFull:
    voltage = _config.holdVoltage;
    break;
  case IntakeState::kOuttaking:
    voltage = _config.outtakeVoltage;
    /* If we no longer have a gear, move to kIdle */
    if (!_config.gearPresenceSensor->Get())
      _state = IntakeState::kIdle;
    break;
  }

  _config.gearbox.transmission->SetVoltage(voltage);
}

void Intake::SetIdle() {
  _state = IntakeState::kIdle;
}

void Intake::SetIntaking() {
  _state = IntakeState::kIntaking;
}

void Intake::SetOuttaking() {
  _state = IntakeState::kOuttaking;
}

IntakeState Intake::GetState() const {
  return _state;
}