#include "Gripper.h"

Gripper::Gripper(GripperConfig config) : _config(config) {}

void Gripper::OnUpdate(units::second_t dt) {
  units::volt_t voltage;

  switch (_state) {
    case GripperState::kIdle:
      voltage = 0_V;
      break;

    case GripperState::kIntaking:
      voltage = 8_V;
      break;

    case GripperState::kOutaking:
      voltage = _speed * -7_V;
      break;  

    case GripperState::kHolding:
      voltage = 3_V;
      break;
  }

  _config.gripperMotor->SetVoltage(voltage);
}

void Gripper::SetIdle() {
  _state = GripperState::kIdle;
}

void Gripper::SetIntaking() {
  _state = GripperState::kIntaking;
}

void Gripper::SetOutaking(double speed) {
  _speed = speed;
  _state = GripperState::kOutaking;
}

void Gripper::SetHolding() {
  _state = GripperState::kHolding;
}

std::string Gripper::GetState() {
  switch (_state) {
  case GripperState::kIdle:
    return "Idle";
    break;
  case GripperState::kHolding:
    return "Holding";
    break;
  case GripperState::kIntaking:
    return "Intaking";
    break;
  case GripperState::kOutaking:
    return "Outaking";
    break;
  }
}