#include "Armavator.h"
#include <units/math.h>

// Your code here
Armavator::Armavator(ArmavatorConfig config)
: config(config), arm(config.arm), elevator(config.elevator) {}

void Armavator::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};

  switch(_state) {
    case ArmavatorState::kIdle:
      break;
    case ArmavatorState::kPosition:
      arm.SetAngle(_setpoint.angle);
      elevator.SetPID(_setpoint.height);
      break;
  }

  arm.OnUpdate(dt);
  elevator.OnUpdate(dt);
}

void Armavator::SetIdle() {
  _state = ArmavatorState::kIdle;
}

void Armavator::SetPosition(ArmavatorPosition pos) {
  _state = ArmavatorState::kPosition;
  _setpoint = pos;
}

ArmavatorPosition Armavator::GetCurrentPosition() const {
  return ArmavatorPosition {
    elevator.GetHeight(),
    arm.GetAngle()
  };
}

bool Armavator::IsStable() const {
  return elevator.IsStable() && arm.IsStable();
}

/* SIMULATION */

::sim::ArmavatorSim::ArmavatorSim(ArmavatorConfig config)
  : config(config), armSim(config.arm), elevatorSim(config.elevator) {}

void ::sim::ArmavatorSim::OnUpdate(units::second_t dt) {
  armSim.Update(dt);
  elevatorSim.Update(dt);
}

units::ampere_t sim::ArmavatorSim::GetCurrent() const {
  return armSim.GetCurrent() + elevatorSim.GetCurrent();
}