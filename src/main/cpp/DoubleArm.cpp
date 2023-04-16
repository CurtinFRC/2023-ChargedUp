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

bool DoubleArm::IsStable() {
  return _extensionArm->IsStable() && _baseArm->IsStable();
}

void DoubleArm::OnStart() {
  _config->baseControl.encoder->SetEncoderPosition(_zeroed.baseAngle);
  _config->connectorControl.encoder->SetEncoderPosition(_zeroed.extensionAngle);
}

void DoubleArm::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};

  switch(_state) {
    case DoubleArmState::kIdle:
      break;

    case DoubleArmState::kPos:
      _config->baseArm.SetAngle(_setpoint.baseAngle);
      _config->extensionArm.SetAngle(_setpoint.extensionAngle);
      break;

    case DoubleArmState::kManual:
      // do math later bc u have to do kinematics or some crap idk
      break;
    
    case DoubleArmState::kRaw:
      _config->baseArm.SetAngle(_inputAngleBase);
      _config->extensionArm.SetAngle(_inputAngleExtension);
      break;

    _config->baseArm.OnUpdate(dt);
    _config->extensionArm.OnUpdate(dt);
  }
}