#include "Arm.h"

using namespace frc;
using namespace wom;

Arm::Arm(ArmConfig config) : _config(config) {   
}

void Arm::OnUpdate(units::second_t dt) {
  units::volt_t voltage = 0_V;
  switch (_state) {
    case ArmState::kIdle:
      break;
    case ArmState::kZeroing:
      voltage = -2_V;
      if (_config.bottomLimitSwitch->Get()) {
        _config.gearbox.encoder->ZeroEncoder();
        _state = ArmState::kIdle;
      }
      break;
    case ArmState::kAngle:
      {
        units::radian_t currentAngle = _config.gearbox.encoder->GetEncoderPosition();
        units::radian_t error = _targetAngle - currentAngle;
        voltage = 12_V / 20_deg * error;
      }
      break;
  }
  _config.gearbox.transmission->SetVoltage(voltage);
}

void Arm::SetIdle() {
  _state = ArmState::kIdle;
}

void Arm::SetZeroing() {
  _state = ArmState::kZeroing;
}

void Arm::SetAngle(units::radian_t angle) {
  _state = ArmState::kAngle;
  _targetAngle = angle;
}