#include "Armavator.h"
#include <units/math.h>

// Your code here

void ArmavatorConfig::WriteNT(std::shared_ptr<nt::NetworkTable> table) {
  //possibleStuff
}

void Armavator::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};

  switch(_state) {
    
  }
}

void Armavator::SetIdle() {
  _state = ArmavatorState::kIdle;
}

void Armavator::SetPosition() {
  _state = ArmavatorState::kPosition;
}

void Armavator::SetZeroing() {
  _state = ArmavatorState::kZeroing;
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