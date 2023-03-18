#pragma once

#include "Arm.h"
#include "Elevator.h"
#include "Gearbox.h"
#include "Grid.h"
#include "drivetrain/SwerveDrive.h"

#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <frc/simulation/ElevatorSim.h>
#include <units/velocity.h>
#include <ctre/Phoenix.h>
#include <units/math.h>
#include "behaviour/HasBehaviour.h"

//the config class
struct ArmavatorConfig {
  using grid_t = wom::DiscretisedOccupancyGrid<units::radian, units::meter>;

  //uses the configs from gthe arm and elevator, as well as includes the grid
  wom::ArmConfig arm;
  wom::ElevatorConfig elevator;
  grid_t grid;
};

//class of info for setting positions
struct ArmavatorPosition {
  units::meter_t height;
  units::radian_t angle;

};

//creates the states used to control the robot
enum class ArmavatorState {
  kIdle,
  kPosition,
  kManual
};

//the behaviour class information
class Armavator : public behaviour::HasBehaviour {
 public:
  Armavator(wom::Gearbox &leftArmGearbox, wom::Gearbox &rightArmGearbox, wom::Gearbox &leftElevatorGearbox, wom::Gearbox &rightElevatorGearbox, ArmavatorConfig &config);
  ~Armavator();

  void OnStart();
  void OnUpdate(units::second_t dt);

  //sets what infomation is needed for the states
  void SetIdle();
  void SetPosition(ArmavatorPosition pos);
  void SetZeroing();
  void SetManual(units::volt_t arm, units::volt_t elevator);
  void SetSpeedValues(double elevatorSpeed, double armSpeed);

  ArmavatorPosition GetCurrentPosition() const;
  bool IsStable() const;

  //creates the arm and the elevator
  wom::Arm *arm;
  wom::Elevator *elevator;
  ArmavatorPosition _setpoint;

 private: 
  ArmavatorState _state = ArmavatorState::kIdle;

  units::volt_t _rawArm;
  units::volt_t _rawElevator;

  //creates an instance of the gearboxes and config
  wom::Gearbox &_leftArmGearbox;
  wom::Gearbox &_rightArmGearbox;
  wom::Gearbox &_leftElevatorGearbox;
  wom::Gearbox &_rightElevatorGearbox;
  ArmavatorConfig &_config;
};