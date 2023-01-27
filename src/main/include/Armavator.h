#pragma once

#include "Arm.h"
#include "Elevator.h"
#include "Gearbox.h"
#include "Grid.h"

#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <frc/simulation/ElevatorSim.h>
#include <units/velocity.h>
#include <ctre/Phoenix.h>
#include <units/math.h>
#include "behaviour/HasBehaviour.h"


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
  kManual
};

class Armavator : public behaviour::HasBehaviour {
 public:
  Armavator(wom::Gearbox &armGearbox, wom::Gearbox &elevatorGearbox, ArmavatorConfig &config);
  ~Armavator();

  void OnUpdate(units::second_t dt);

  void SetIdle();
  void SetPosition(ArmavatorPosition pos);
  void SetZeroing();
  void SetManual(ArmavatorPosition pos);

  

  ArmavatorPosition GetCurrentPosition() const;
  bool IsStable() const;

  wom::Arm *arm;
  wom::Elevator *elevator;

 private: 
  ArmavatorState _state = ArmavatorState::kIdle;

  ArmavatorPosition _setpoint;

  wom::Gearbox &_armGearbox;
  wom::Gearbox &_elevatorGearbox;
  ArmavatorConfig &_config;
};
