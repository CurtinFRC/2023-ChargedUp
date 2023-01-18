// #include "SwerveMod.h"
// #include <units/math.h>
// #include "RobotMap.h"

// SwerveModuleTest::SwerveModuleTest(SwerveSingleModuleConfig &motors) : _motors(motors) {}

// void SwerveModuleTest::OnUpdate(units::second_t dt) {
//   double driveSpeed, turnSpeed;
//   switch(SwerveModuleTest::_state){
//     case SwerveModuleStates::kIdle:
//       driveSpeed = 0;
//       turnSpeed = 0;
//       break;
//     case SwerveModuleStates::kRaw: 
//       driveSpeed = _driveSpeed;
//       turnSpeed = _turnSpeed;
//       break;

      
//   // _driverMotor.Set(driveSpeed);
//   // _turnMotor.Set(turnSpeed);
//   _motors.driveMotor.Set(driveSpeed);
//   _motors.turnMotor.Set(turnSpeed);
//   }
  
// }

// void SwerveModuleTest::SetRaw(double driveSpeed, double turnSpeed){
//   _state = SwerveModuleStates::kRaw;
//   _driveSpeed = driveSpeed;
//   _turnSpeed = turnSpeed;

// }

// void SwerveModuleTest::SetIdle(){
//   _state = SwerveModuleStates::kIdle;
// }

// void SwerveModuleTest::ReadoutEncoderValues(){
//   _table = nt::NetworkTableInstance::GetDefault().GetTable("");
//   _table->GetEntry("driverEncoderPosition").SetDouble((new wom::TalonFXEncoder(new WPI_TalonFX(1)))->GetEncoderPosition().value()); // (1) is the first motor value in SwerveSingleModule in RobotMap.h
//   _table->GetEntry("turnEncoderPosition").SetDouble((new wom::TalonFXEncoder(new WPI_TalonFX(2)))->GetEncoderPosition().value()); // (2) is the second motor value in SwerveSingleModule in RobotMap.h

// }