#pragma once

#include "Arm.h"
#include "Elevator.h"
#include "Gearbox.h"
#include "Grid.h"

#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <frc/simulation/ElevatorSim.h>
#include <units/velocity.h>

struct ArmavatorConfig {
  using grid_t = wom::DiscretisedOccupancyGrid<units::radian, units::meter>;

  wom::ArmConfig arm;
  wom::ElevatorConfig elevator;
  grid_t grid;
};

struct ArmavatorPosition {
  units::meter_t height;
  units::radian_t angle;
};


enum class ArmavatorState {
  kIdle,
  kPosition,
  //ManualPositioning
};

class Armavator : public behaviour::HasBehaviour {
 public:
  Armavator(ArmavatorConfig config);

  void OnUpdate(units::second_t dt);

  void SetIdle();
  void SetPosition(ArmavatorPosition pos);

  ArmavatorPosition GetCurrentPosition() const;
  bool IsStable() const;

  ArmavatorConfig config;
  wom::Arm arm;
  wom::Elevator elevator;

 private: 
  ArmavatorState _state = ArmavatorState::kIdle;

  ArmavatorPosition _setpoint;
};

/* SIMULATION */

namespace sim {
  class ArmavatorSim {
   public:
    ArmavatorSim(ArmavatorConfig config);

    void OnUpdate(units::second_t dt);

    units::ampere_t GetCurrent() const;

    ArmavatorConfig config;

    wom::sim::ArmSim armSim;
    wom::sim::ElevatorSim elevatorSim;
  };
}