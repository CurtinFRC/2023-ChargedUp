#include "SideIntake.h"

using namespace frc;
using namespace wom;

SideIntake::SideIntake(SideIntakeConfig config) : _config(config) {}

void SideIntake::OnUpdate(units::second_t dt) {
  units::volt_t voltage = 0_V;

  switch (_state) {
<<<<<<< HEAD
  case SideIntakeState::kIdle:
    voltage = 0_V;
    break;

  case SideIntakeState::kStowed:
    _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    voltage = 0_V;
    break;

  case SideIntakeState::kIntakingWide:
    _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kReverse);

    voltage = 10_V;
    break;

  case SideIntakeState::kOuttakingWide:
    _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kReverse);

    voltage = -10_V;
    break;

  case SideIntakeState::kIntakingClosed:
    _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kForward);

    voltage = 10_V;
    break;

  case SideIntakeState::kOutakingClosed:
    _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kForward);

    voltage = -10_V;
    break;

  case SideIntakeState::kWide:
    _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    voltage = 0_V;
    break;
  case SideIntakeState::kClosed:
    _config.claspSolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    _config.deploySolenoid->Set(frc::DoubleSolenoid::Value::kForward);
    voltage = 0_V;
    break;
=======
    case SideIntakeState::kIdle:
      voltage = 0_V;
      break;

    case SideIntakeState::kIntaking:
        voltage = intakeVoltage;
        //_config.claspSolenoid->Set(frc::DoubleSolenoid::kForward);
      break;
    
    case SideIntakeState::kMovePiston:
      _config.deploySolenoid->Toggle();
      break;

    case SideIntakeState::kClaspPiston:
    _config.claspSolenoid->Toggle();
     break;

    case SideIntakeState::kOuttaking:
        voltage = outtakeVoltage;
        //_config.claspSolenoid->Set(frc::DoubleSolenoid::kReverse);
       
        
      
      break;
>>>>>>> 5bd0db5b1a3254005a6079342795f677ee67d364
  }

    _config.leftIntakeMotor->SetVoltage(-voltage);
    _config.rightIntakeMotor->SetVoltage(voltage);
};

void SideIntake::SetIdle() {
  _state = SideIntakeState::kIdle;
}

void SideIntake::SetIntakingWide() {
  _state = SideIntakeState::kIntakingWide;
}

<<<<<<< HEAD
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
=======
void SideIntake::SetMovePiston() {
  _state = SideIntakeState::kMovePiston;
}

void SideIntake::SetClaspPiston(){
  _state = SideIntakeState::kClaspPiston;
}
void SideIntake::SetOuttaking() {
  _state = SideIntakeState::kOuttaking;
}



SideIntakeState SideIntake::GetState() const {
  return _state;
>>>>>>> 5bd0db5b1a3254005a6079342795f677ee67d364
}