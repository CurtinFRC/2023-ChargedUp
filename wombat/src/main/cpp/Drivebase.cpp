#include "Drivebase.h"

using namespace wom;

void Drivetrain::Set(double leftPower, double rightPower) {
  SetVoltage(leftPower * 12, rightPower * 12);
}

void Drivetrain::SetVoltage(double left, double right) {

}

void Drivetrain::SetInverted(bool inverted = false) {

}

Gearbox &Drivetrain::GetLeft() {
  return !_config.reversed ? _config.leftDrive : _config.rightDrive;
}

Gearbox &Drivetrain::GetRight() {
  return !_config.reversed ? _config.rightDrive : _config.leftDrive;
}