#include "DoubleArm.hpp"


void DoubleArm::SetIdle() {
  _state = DoubleArmState::kIdle; 
}

void DoubleArm::SetPosition(DoubleArmPos pos) {
  _state = DoubleArmState::kPos;
  _setpoint = pos;
}

void DoubleArm::SetZeroing() {
  _state = DoubleArmState::kPos;
  _setpoint = _zeroed;
  _state = DoubleArmState::kIdle;
}

void DoubleArm::SetManual(units::volt_t baseArmRaw, units::volt_t extensionArmRaw) {
  _state = DoubleArmState::kManual;
  _baseArmRaw = baseArmRaw;
  _extensionArmRaw = extensionArmRaw;
}


void DoubleArm::SetSpeedValues(double baseArmSpeed, double extensionArmSpeed) {
  _baseArm->SetArmSpeedLimit(baseArmSpeed);
  _extensionArm->SetArmSpeedLimit(extensionArmSpeed);
}

DoubleArmPos DoubleArm::GetCurrentPos() {
  return DoubleArmPos {
      _baseArm->GetAngle(),
      _extensionArm->GetAngle()
  };
}