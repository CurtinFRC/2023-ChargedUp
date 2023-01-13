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

  void WriteNT(std::shared_ptr<nt::NetworkTable> table);
};

// Your code here
enum class ArmavatorState {
  kIdle,
  kPosition,
  kZeroing
  //ManualPositioning
};

class Armavator : public behaviour::HasBehaviour {
 public:
  Armavator(ArmavatorConfig config);

  void OnUpdate(units::second_t dt);

  void SetIdle();
  void SetZeroing();
  void SetPosition();

 private:
  ArmavatorConfig _config;
  ArmavatorState _state;

  wom::PIDController<units::radian, units::volt> _pid;
  std::shared_ptr<nt::NetworkTable> _table;

  wom::Arm arm;
  wom::Elevator elevator;
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