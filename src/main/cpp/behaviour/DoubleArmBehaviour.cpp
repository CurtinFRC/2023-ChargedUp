#include "behaviour/DoubleArmBehaviour.hpp"

DoubleArmGoToAutoSetPoint::DoubleArmGoToAutoSetPoint(DoubleArm *doubleArm, units::radian_t baseAngle, units::radian_t extensionAngle, double extensionSpeed, double baseSpeed)
: _doubleArm(doubleArm), _baseAngle(baseAngle), _extensionAngle(extensionAngle), _extensionSpeed(extensionSpeed), _baseSpeed(baseSpeed) {
  Controls(doubleArm);
}

void DoubleArmGoToAutoSetPoint::OnStart() {
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
  _doubleArm->SetZeroing();
  std::cout << "Raw Control behaviour started" << std::endl;
};

void RawControlBehaviour::OnTick(units::second_t dt) {
  double basePower = std::abs(_codriver.GetLeftY()) > 0.15 ? _codriver.GetLeftY() + 0.1 : 0;
  double extensionPower = std::abs(_codriver.GetLeftX()) > 0.15 ? _codriver.GetLeftX() + 0.1 : 0;

  _doubleArm->GetConfig()->baseControl.transmission->SetVoltage(12_V * basePower);
  _doubleArm->GetConfig()->connectorControl.transmission->SetVoltage(12_V * extensionPower);
};