#include "SwerveMod.h"
#include <units/math.h>

SwerveModuleTest::SwerveModuleTest(TalonFX driveMotor, TalonFX turnMotor) : _driverMotor(driveMotor), _turnMotor(turnMotor)  {}

void SwerveModuleTest::OnUpdate(units::second_t dt) {
  double driveSpeed, turnSpeed;
  switch(SwerveModuleTest::_state){
    case SwerveModuleStates::kIdle:
      driveSpeed = 0;
      turnSpeed = 0;
      break;
    case SwerveModuleStates::kRaw: 
      driveSpeed = _driveSpeed;
      turnSpeed = _turnSpeed;
      break;

      
  _driverMotor.Set(TalonFXControlMode::PercentOutput, driveSpeed);
  _turnMotor.Set(TalonFXControlMode::PercentOutput, turnSpeed);
  }
  
}

void SwerveModuleTest::SetRaw(double driveSpeed, double turnSpeed){
  _state = SwerveModuleStates::kRaw;
  _driveSpeed = driveSpeed;
  _turnSpeed = turnSpeed;

}

void SwerveModuleTest::SetIdle(){
  _state = SwerveModuleStates::kIdle;
}