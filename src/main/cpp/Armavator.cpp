#include "Armavator.h"

//Armavator configeration
Armavator::Armavator(wom::Gearbox &leftArmGearbox, wom::Gearbox &rightArmGearbox , wom::Gearbox &rightElevatorGearbox ,wom::Gearbox &leftElevatorGearbox, ArmavatorConfig &config)
: _leftArmGearbox(leftArmGearbox), _rightArmGearbox(rightArmGearbox), _rightElevatorGearbox(rightElevatorGearbox), _leftElevatorGearbox(leftElevatorGearbox),_config(config) {
  arm = new wom::Arm(config.arm); //create a new wombat arm
  elevator = new wom::Elevator(config.elevator); //create a new wombat elevator 
}

Armavator::~Armavator() {
  free(arm);
  free(elevator);
}

void Armavator::OnStart() {
  std::cout << "STARTING" << std::endl;
}

void Armavator::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0}; 

  switch(_state) {
    case ArmavatorState::kIdle: //armavator should do nothing when in the idle state
      break;
    case ArmavatorState::kVelocity: //sets both the arm and the elevator to the stored velocity's 
      arm->SetVelocity(_velocitySetpoint.angleSpeed);
      elevator->SetVelocity(_velocitySetpoint.elevatorSpeed);
      break;
    case ArmavatorState::kPosition: //sets the arm and elevator to the stored positions 
      arm->SetAngle(_setpoint.angle);
      elevator->SetPID(_setpoint.height);
      break;
    case ArmavatorState::kManual: //sets the arm and elevator to the stored speeds
      arm->SetRaw(_rawArm);
      elevator->SetManual(_rawElevator);
      break;
  }

  arm->OnUpdate(dt);
  elevator->OnUpdate(dt);
}

//Sets the armavator to the idle state 
void Armavator::SetIdle() {
  _state = ArmavatorState::kIdle;
}

//set the armavator to the position state and stores the desired position
void Armavator::SetPosition(ArmavatorPosition pos) {
  _state = ArmavatorState::kPosition;
  _setpoint = pos;
}

//sets the armavator to the manual state and stores the voltages
void Armavator::SetManual(units::volt_t arm, units::volt_t elevator) {
  _state = ArmavatorState::kManual;
  _rawArm = arm;
  _rawElevator = elevator;
}

//Sets the armavator to the velocity state and stores the desired voltages
void Armavator::SetVelocity(ArmavatorVelocity vel) {
  _state = ArmavatorState::kVelocity;
  _velocitySetpoint = vel;
}

//sets a multiplier to the arm and elevator speeds, anything less than 1 will slow it down, anything more will speed it up
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