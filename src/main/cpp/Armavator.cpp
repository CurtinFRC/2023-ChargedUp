#include "Armavator.h"
#include <units/math.h>

// Your code here
Armavator::Armavator(ArmavatorConfig config)
: _config(config), arm(config.arm), elevator(config.elevator) {}

void Armavator::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};

  switch(_state) {
    case ArmavatorState::kIdle:
      break;
    case ArmavatorState::kPosition:
      arm.SetAngle(_armSetpoint);
      elevator.SetPID(_elevatorSetpoint);
      break;
  }

  arm.OnUpdate(dt);
  elevator.OnUpdate(dt);
}

void Armavator::SetIdle() {
  _state = ArmavatorState::kIdle;
}

void Armavator::SetPosition(units::meter_t elevatorHeight, units::radian_t armAngle) {
  _state = ArmavatorState::kPosition;
  _armSetpoint = armAngle;
  _elevatorSetpoint = elevatorHeight;
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