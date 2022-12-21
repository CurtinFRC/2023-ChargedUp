#include "Drivebase.h"

using namespace wom;

Drivetrain::Drivetrain(DrivetrainConfig config) : _config(config) {}

void Drivetrain::OnUpdate(units::second_t dt) {
  units::volt_t leftVoltage{0};
  units::volt_t rightVoltage{0};

  switch(_state) {
    case DrivetrainState::kIdle:
      leftVoltage = 0_V;
      rightVoltage = 0_V;
      break;
    case DrivetrainState::kManual:
      leftVoltage = _leftSetPointManual;
      rightVoltage = _rightSetPointManual;
      break;
    case DrivetrainState::kTurnToAngle:

      break;
    case DrivetrainState::kDriveToDistance:
      break;
    case DrivetrainState::kSpline: 
      break;
  }
}

void Drivetrain::Set(double leftPower, double rightPower) {
  SetVoltage(leftPower * 12, rightPower * 12);
}

void Drivetrain::SetVoltage(double left, double right) {

}

void Drivetrain::SetInverted(bool inverted) {
  if (inverted != _config.reversed) {
    _config.reversed = inverted;
    _config.leftDrive.transmission->SetInverted(!_config.leftDrive.transmission->GetInverted());
    _config.rightDrive.transmission->SetInverted(!_config.rightDrive.transmission->GetInverted());
  }
}

Gearbox &Drivetrain::GetLeft() {
  return !_config.reversed ? _config.leftDrive : _config.rightDrive;
}

Gearbox &Drivetrain::GetRight() {
  return !_config.reversed ? _config.rightDrive : _config.leftDrive;
}

void Drivetrain::TurnToAngle(double goal, units::second_t dt) {
  
}