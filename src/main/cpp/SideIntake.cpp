#include "SideIntake.h"

using namespace frc;
using namespace wom;

SideIntake::SideIntake(SideIntakeConfig config) : _config(config) {}

void SideIntake::OnUpdate(units::second_t dt) {
  units::volt_t voltage = 0_V;

  // switch (_state) {
  // case SideIntakeState::kIdle:
  //   voltage = 0_V;
  //   break;

  // case SideIntakeState::kStowed:
  //   _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
  //   _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
  //   voltage = 0_V;
  //   break;

  // case SideIntakeState::kIntakingWide:
  //   _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
  //   _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kReverse);

  //   voltage = 10_V;
  //   break;

  // case SideIntakeState::kOuttakingWide:
  //   _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
  //   _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kReverse);

  //   voltage = -10_V;
  //   break;

  // case SideIntakeState::kIntakingClosed:
  //   _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
  //   _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kForward);

  //   voltage = 10_V;
  //   break;

  // case SideIntakeState::kOutakingClosed:
  //   _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
  //   _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kForward);

  //   voltage = -10_V;
  //   break;

  // case SideIntakeState::kWide:
  //   _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
  //   _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
  //   voltage = 0_V;
  //   break;
  // case SideIntakeState::kClosed:
  //   _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
  //   _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kForward);
  //   voltage = 0_V;
  //   break;
  // }

  switch (_actuationState) {
    case IntakeActuationState::kDeployed:
      _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
      break;
    
    case IntakeActuationState::kStowed:
      _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kForward);
      break;
  }

  switch (_closedState) {
    case IntakeCloseState::kOpen:
      _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
      break;
    
    case IntakeCloseState::kClosed:
      _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
      break;
  }

    _config.leftGearbox->transmission->SetVoltage(-_voltage);
    _config.rightGearbox->transmission->SetVoltage(_voltage);
};

void SideIntake::SetIdle() {
  _state = SideIntakeState::kIdle;
}

void SideIntake::SetIntakingWide() {
  _state = SideIntakeState::kIntakingWide;
}

void SideIntake::SetOutakingWide() {
  _state = SideIntakeState::kOuttakingWide;
}

void SideIntake::SetIntakingClosed() {
  _state = SideIntakeState::kIntakingClosed;
}

void SideIntake::SetOutakingClosed() {
  _state = SideIntakeState::kOutakingClosed;
}

void SideIntake::SetStowed() {
  _state = SideIntakeState::kStowed;
}

void SideIntake::SetWide() {
  _state = SideIntakeState::kWide;
}

void SideIntake::SetClosed() {
  _state = SideIntakeState::kClosed;
}

void SideIntake::SetStow() {
  _actuationState = IntakeActuationState::kStowed;
}

void SideIntake::SetDeploy() {
  _actuationState = IntakeActuationState::kDeployed;
}

void SideIntake::SetClose() {
  _closedState = IntakeCloseState::kClosed;
}

void SideIntake::SetOpen() {
  _closedState = IntakeCloseState::kOpen;
}

void SideIntake::SetVoltage(units::volt_t voltage) {
  _voltage = voltage;
}

std::string SideIntake::GetState() const {
  switch (_state) {
  case SideIntakeState::kIdle:
    return "Idle";
    break;
  case SideIntakeState::kIntakingWide:
    return "Intaking Wide";
    break;
  case SideIntakeState::kIntakingClosed:
    return "Intaking Closed";
    break;
  case SideIntakeState::kOutakingClosed:
    return "Outaking Closed";
    break;
  case SideIntakeState::kOuttakingWide:
    return "Outaking Wide";
    break;
  case SideIntakeState::kWide:
    return "Wide";
    break;
  case SideIntakeState::kClosed:
    return "Closed";
    break;
  case SideIntakeState::kStowed:
    return "Stowed";
    break;
  }
}