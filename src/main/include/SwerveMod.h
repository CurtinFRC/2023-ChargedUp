// #pragma once

// #include "Gearbox.h"
// #include "Grid.h"
// #include "behaviour/HasBehaviour.h"
// #include <frc/DigitalInput.h>
// #include <frc/simulation/DIOSim.h>
// #include <frc/simulation/ElevatorSim.h>
// #include <units/velocity.h>
// #include "networktables/NetworkTable.h"


// struct SwerveSingleModuleConfig{
//   WPI_TalonFX driveMotor;
//   WPI_TalonFX turnMotor;
// };

// enum class SwerveModuleStates {
//   kIdle,
//   kRaw
// };

// class SwerveModuleTest : public behaviour::HasBehaviour {
//  public:
//   SwerveModuleTest(SwerveSingleModuleConfig &motors);

//   void OnUpdate(units::second_t dt);
//   void SetRaw(double driveSpeed, double turnSpeed);
//   void SetIdle();
//   void ReadoutEncoderValues();

//  private:
//   double _driveSpeed = 0;
//   double _turnSpeed = 0;
//   SwerveModuleStates _state = SwerveModuleStates::kIdle;
//   std::shared_ptr<nt::NetworkTable> _table;
//   SwerveSingleModuleConfig &_motors;
//   // WPI_TalonFX &_driverMotor;
//   // WPI_TalonFX &_turnMotor;
  

// };
