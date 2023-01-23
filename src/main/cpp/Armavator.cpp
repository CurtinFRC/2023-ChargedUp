#include "Armavator.h"

//Armavator configeration
Armavator::Armavator(wom::Gearbox &armGearbox, wom::Gearbox &elevatorGearbox, ArmavatorConfig &config)
: _armGearbox(armGearbox), _elevatorGearbox(elevatorGearbox), _config(config) {
  arm = new wom::Arm(config.arm);
  elevator = new wom::Elevator(config.elevator);
}

Armavator::~Armavator() {
  free(arm);
  free(elevator);
}

//Instructions for when the program updates (seconds delta time)
void Armavator::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};

  switch(_state) {
    case ArmavatorState::kIdle:
      break;
    case ArmavatorState::kPosition:
      arm->SetAngle(_setpoint.angle);
      elevator->SetPID(_setpoint.height);
      break;
  }

  arm->OnUpdate(dt);
  elevator->OnUpdate(dt);
}
//Sets the states names
void Armavator::SetIdle() {
  _state = ArmavatorState::kIdle;
}

void Armavator::SetPosition(ArmavatorPosition pos) {
  _state = ArmavatorState::kPosition;
  _setpoint = pos;
}

ArmavatorPosition Armavator::GetCurrentPosition() const {
  return ArmavatorPosition {
    elevator->GetHeight(),
    arm->GetAngle()
  };
}

bool Armavator::IsStable() const {
  return elevator->IsStable() && arm->IsStable();
}