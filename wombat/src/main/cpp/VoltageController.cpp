#include "VoltageController.h"
#include <frc/RobotController.h>

using namespace wom;

MotorVoltageController::MotorVoltageController(frc::MotorController *MotorController) : _MotorController(MotorController) {}

void MotorVoltageController::SetVoltage(units::volt_t voltage) {
  _MotorController->Set(voltage / GetBusVoltage());
}

units::volt_t MotorVoltageController::GetVoltage() const {
  return _MotorController->Get() * GetBusVoltage();
}

units::volt_t MotorVoltageController::GetBusVoltage() const {
  return frc::RobotController::GetInputVoltage() * 1_V;
}

void MotorVoltageController::SetInverted(bool invert) {
  _MotorController->SetInverted(invert);
}

bool MotorVoltageController::GetInverted() const {
  return frc::RobotController::GetInputVoltage();
}