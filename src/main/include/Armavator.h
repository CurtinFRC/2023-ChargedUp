#pragma once

#include "Arm.h"
#include "Elevator.h"
#include "Gearbox.h"

#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <frc/simulation/ElevatorSim.h>
#include <units/velocity.h>

struct ArmavatorConfig {
  wom::ArmConfig arm;
  wom::ElevatorConfig elevator;
};

// Your code here

/* SIMULATION */

namespace sim {
  class ArmavatorSim {
   public:
    ArmavatorSim(ArmavatorConfig config);

    void Update(units::second_t dt);

    units::ampere_t GetCurrent() const;

    ArmavatorConfig config;

    wom::sim::ArmSim armSim;
    wom::sim::ElevatorSim elevatorSim;
  };
}