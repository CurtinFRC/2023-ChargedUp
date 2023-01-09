#include "Armavator.h"

// Your code here

/* SIMULATION */
#include <units/math.h>

::sim::ArmavatorSim::ArmavatorSim(ArmavatorConfig config)
  : config(config), armSim(config.arm), elevatorSim(config.elevator) {}

void ::sim::ArmavatorSim::Update(units::volt_t elevatorVoltage, units::volt_t armVoltage, units::second_t dt) {
  armSim.Update(armVoltage, dt);
  elevatorSim.Update(elevatorVoltage, dt);
}