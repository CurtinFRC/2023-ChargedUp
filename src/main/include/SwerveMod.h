#pragma once

#include "Gearbox.h"
#include "Grid.h"
#include "behaviour/HasBehaviour.h"
#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <frc/simulation/ElevatorSim.h>
#include <units/velocity.h>


enum class SwerveModuleStates {
  kIdle,
  kRaw
};

class SwerveModuleTest : public behaviour::HasBehaviour {
 public:
  SwerveModuleTest(TalonFX driveMotor, TalonFX turnMotor);

  void OnUpdate(units::second_t dt);
  void SetRaw(double driveSpeed, double turnSpeed);
  void SetIdle();

 private:
  double _driveSpeed = 0;
  double _turnSpeed = 0;
  SwerveModuleStates _state = SwerveModuleStates::kIdle;
  TalonFX &_driverMotor;
  TalonFX &_turnMotor;
    
};
