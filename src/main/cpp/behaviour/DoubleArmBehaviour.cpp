#include "behaviour/DoubleArmBehaviour.hpp"
#include "ControlUtil.h"

DoubleArmGoToAutoSetPoint::DoubleArmGoToAutoSetPoint(DoubleArm *doubleArm, units::radian_t baseAngle, units::radian_t extensionAngle, double extensionSpeed, double baseSpeed)
: _doubleArm(doubleArm), _baseAngle(baseAngle), _extensionAngle(extensionAngle), _extensionSpeed(extensionSpeed), _baseSpeed(baseSpeed) {
  Controls(doubleArm);
}

void DoubleArmGoToAutoSetPoint::OnStart() {
  _doubleArm->OnStart();
  std::cout << "starting auto setpoint behaviour double arm" << std::endl;
}

void DoubleArmGoToAutoSetPoint::OnTick(units::second_t dt) {
  DoubleArmPos pos = {_baseAngle, _extensionAngle};
  _doubleArm->SetSpeedValues(_baseSpeed, _extensionSpeed);

  _doubleArm->SetPosition(pos);
  if (_doubleArm->IsStable() == true) {
    SetDone();
  }
}

RawControlBehaviour::RawControlBehaviour(DoubleArm *doubleArm, frc::XboxController &codriver)
: _doubleArm(doubleArm), _codriver(_codriver) { Controls(doubleArm); }

void RawControlBehaviour::OnStart() {
  _doubleArm->OnStart();
  _doubleArm->SetZeroing();
  std::cout << "Raw Control behaviour started" << std::endl;
};

void RawControlBehaviour::OnTick(units::second_t dt) {
  double basePower = std::abs(_codriver.GetLeftY()) > 0.15 ? _codriver.GetLeftY() + 0.1 : 0;
  double extensionPower = std::abs(_codriver.GetLeftX()) > 0.15 ? _codriver.GetLeftX() + 0.1 : 0;

  _doubleArm->GetConfig()->baseControl.transmission->SetVoltage(12_V * basePower);
  _doubleArm->GetConfig()->connectorControl.transmission->SetVoltage(12_V * extensionPower);
};

ManualControlBehaviour::ManualControlBehaviour(DoubleArm *doubleArm, frc::XboxController &codriver)
: _doubleArm(doubleArm), _codriver(codriver) {}

void ManualControlBehaviour::OnStart() {
  _doubleArm->OnStart();
  _doubleArm->SetZeroing();
  std::cout << "Manual behaviour started" << std::endl;
}

void ManualControlBehaviour::OnTick(units::second_t dt) {
  _doubleArm->GetEndEffectorPos(_doubleArm->GetConfig()->baseArm.GetConfig().armLength, _doubleArm->GetConfig()->extensionArm.GetConfig().armLength, _doubleArm->GetConfig()->baseArm.GetAngle(), _doubleArm->GetConfig()->extensionArm.GetAngle());
  ConfigSpacePos ControllerChange {
    _codriver.GetLeftX() * 1_m,
    _codriver.GetLeftY() * 1_m
  };
  units::radian_t baseAngle = _doubleArm->GetConfig()->baseArm.GetAngle();
  units::radian_t extensionAngle = _doubleArm->GetConfig()->extensionArm.GetAngle();
  double x = _doubleArm->GetConfig()->baseArm.GetConfig().armLength.value();
  double x2 = wom::spow2(x);
  double y = _doubleArm->GetConfig()->extensionArm.GetConfig().armLength.value();
  double y2 = wom::spow2(y);
  
  ConfigSpacePos endPos = _doubleArm->GetEndEffectorPos(_doubleArm->GetConfig()->baseArm.GetConfig().armLength, _doubleArm->GetConfig()->extensionArm.GetConfig().armLength, _doubleArm->GetConfig()->baseArm.GetAngle(), _doubleArm->GetConfig()->extensionArm.GetAngle());
  double M2 = wom::spow2(endPos.X.value());
  double N2 = wom::spow2(endPos.Y.value());
  units::radian_t baseNewAngle = std::acos((x2 + y2 - wom::spow2(M2 + N2)) / (2 * x * y)) * 1_rad;
  units::radian_t beta = acos((x2 + wom::spow2(M2 + N2) - y2) / (2 * x * y)) * 1_rad;
  units::radian_t alpha = std::atan(endPos.Y.value()/endPos.X.value()) * 1_rad;
  units::radian_t extensionNewAngle = beta + alpha;
  _doubleArm->GetConfig()->baseArm.SetAngle(baseNewAngle);
  _doubleArm->GetConfig()->extensionArm.SetAngle(extensionNewAngle);
}