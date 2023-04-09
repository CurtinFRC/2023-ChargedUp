#include "Armavator.h"

//Armavator configeration
Armavator::Armavator(wom::Gearbox &leftArmGearbox, wom::Gearbox &rightArmGearbox , wom::Gearbox &rightElevatorGearbox ,wom::Gearbox &leftElevatorGearbox, ArmavatorConfig &config)
: _leftArmGearbox(leftArmGearbox), _rightArmGearbox(rightArmGearbox), _rightElevatorGearbox(rightElevatorGearbox), _leftElevatorGearbox(leftElevatorGearbox),_config(config) {
  arm = new wom::Arm(config.arm);
  elevator = new wom::Elevator(config.elevator);
}

Armavator::~Armavator() {
  free(arm);
  free(elevator);
}

void Armavator::OnStart() {
  // _config.elevator.leftGearbox.encoder->ZeroEncoder();
  // _config.elevator.rightGearbox.encoder->ZeroEncoder();

  _config.arm.leftGearbox.encoder->SetEncoderPosition(90_deg);
  _config.arm.rightGearbox.encoder->SetEncoderPosition(90_deg);
  std::cout << "STARTING" << std::endl;
}

//Instructions for when the program updates (seconds delta time)
void Armavator::OnUpdate(units::second_t dt) {
  // std::cout << "ON UPDATE" << std::endl;
  units::volt_t voltage{0};

  // std::cout << elevator->GetElevatorEncoderPos() << std::endl;

  switch(_state) {
    case ArmavatorState::kIdle:
      break;
    case ArmavatorState::kPosition:
      arm->SetAngle(_setpoint.angle);
      elevator->SetPID(_setpoint.height);
      break;
    case ArmavatorState::kManual:
      arm->SetRaw(_rawArm);
      elevator->SetManual(_rawElevator);
      break;
  }

  arm->OnUpdate(dt);
  elevator->OnUpdate(dt);
}

//Sets the states names
//idle state
void Armavator::SetIdle() {
  _state = ArmavatorState::kIdle;
}

//set positions state
void Armavator::SetPosition(ArmavatorPosition pos) {
  _state = ArmavatorState::kPosition;
  _setpoint = pos;
}

//manual state setup
void Armavator::SetManual(units::volt_t arm, units::volt_t elevator) {
  _state = ArmavatorState::kManual;
  _rawArm = arm;
  _rawElevator = elevator;
}

void Armavator::SetSpeedValues(double elevatorSpeed, double armSpeed) {
  arm->SetArmSpeedLimit(armSpeed);
  elevator->SetElevatorSpeedLimit(elevatorSpeed);
}


//returns the current position
ArmavatorPosition Armavator::GetCurrentPosition() const {
  return ArmavatorPosition {
    elevator->GetHeight(),
    arm->GetAngle()
  };
}

//determines if the armavator is stable/done
bool Armavator::IsStable() const {
  return elevator->IsStable() && arm->IsStable();
}