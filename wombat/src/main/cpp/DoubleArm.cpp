#include "DoubleArm.hpp"
#include "ControlUtil.h"


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
  GetEndEffectorPos(GetConfig()->baseArm.GetConfig().armLength, GetConfig()->extensionArm.GetConfig().armLength, GetConfig()->baseArm.GetAngle(), GetConfig()->extensionArm.GetAngle());
  ConfigSpacePos ControllerChange {
    _codriver.GetLeftX() * 1_m,
    _codriver.GetLeftY() * 1_m
  };
  units::radian_t baseAngle = GetConfig()->baseArm.GetAngle();
  units::radian_t extensionAngle = GetConfig()->extensionArm.GetAngle();
  double x = GetConfig()->baseArm.GetConfig().armLength.value();
  double x2 = wom::spow2(x);
  double y = GetConfig()->extensionArm.GetConfig().armLength.value();
  double y2 = wom::spow2(y);
  
  ConfigSpacePos endPos = GetEndEffectorPos(GetConfig()->baseArm.GetConfig().armLength, GetConfig()->extensionArm.GetConfig().armLength, GetConfig()->baseArm.GetAngle(), GetConfig()->extensionArm.GetAngle());
  double M2 = wom::spow2(endPos.X.value());
  double N2 = wom::spow2(endPos.Y.value());
  units::radian_t baseNewAngle = std::acos((x2 + y2 - wom::spow2(M2 + N2)) / (2 * x * y)) * 1_rad;
  units::radian_t beta = acos((x2 + wom::spow2(M2 + N2) - y2) / (2 * x * y)) * 1_rad;
  units::radian_t alpha = std::atan(endPos.Y.value()/endPos.X.value()) * 1_rad;
  units::radian_t extensionNewAngle = beta + alpha;

  switch(_state) {
    case DoubleArmState::kIdle:
      break;

    case DoubleArmState::kPos:
      _config->baseArm.SetAngle(_setpoint.baseAngle);
      _config->extensionArm.SetAngle(_setpoint.extensionAngle);
      break;

    case DoubleArmState::kManual:
      GetConfig()->baseArm.SetAngle(baseNewAngle);
      GetConfig()->extensionArm.SetAngle(extensionNewAngle);      
      break;

    case DoubleArmState::kRaw:
      _config->baseArm.SetAngle(_inputAngleBase);
      _config->extensionArm.SetAngle(_inputAngleExtension);
      break;

    _config->baseArm.OnUpdate(dt);
    _config->extensionArm.OnUpdate(dt);
  }
}