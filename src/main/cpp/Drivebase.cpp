#include "Drivebase.h"

#include "Drivebase.h"

#include <iostream>

using namespace wom;

MecanumDrivebase::MecanumDrivebase(MecanumDrivebaseConfig config)
  : _config(config),
    _kinematics(_config.frontLeftPos, _config.frontRightPos, _config.rearLeftPos, _config.rearRightPos) { }

void MecanumDrivebase::OnUpdate(units::second_t deltaTime){
  units::volt_t flVoltage = 0_V, frVoltage = 0_V;
  units::volt_t rlVoltage = 0_V, rrVoltage = 0_V;

  switch (_activeState) {
  case MecanumDrivebaseState::kIdle:
    break;
  case MecanumDrivebaseState::kManual:
    break;
  case MecanumDrivebaseState::kVelocity:
    {
      frc::MecanumDriveWheelSpeeds wheelSpeeds = _kinematics.ToWheelSpeeds(_targetSpeeds);
      // v = w*r
      // w = v/r
      units::radians_per_second_t flSpeed = 1_rad * wheelSpeeds.frontLeft / _config.wheelRadius;
      units::radians_per_second_t frSpeed = 1_rad * wheelSpeeds.frontRight / _config.wheelRadius;
      units::radians_per_second_t rlSpeed = 1_rad * wheelSpeeds.rearLeft / _config.wheelRadius;
      units::radians_per_second_t rrSpeed = 1_rad * wheelSpeeds.rearRight / _config.wheelRadius;
      // assuming torque = 0Nm
      flVoltage = _config.frontLeftGearBox.motor.Voltage(0_Nm, flSpeed);
      frVoltage = _config.frontRightGearBox.motor.Voltage(0_Nm, frSpeed);
      rlVoltage = _config.rearLeftGearBox.motor.Voltage(0_Nm, rlSpeed);
      rrVoltage = _config.rearRightGearBox.motor.Voltage(0_Nm, rrSpeed);

      std::cout << "Vels " << flVoltage.value() << ", " << frVoltage.value() << ", " << rlVoltage.value() << ", " << rrVoltage.value() << std::endl;
    }
    break;
  }

  _config.frontLeftGearBox.transmission->SetVoltage(flVoltage);
  _config.frontRightGearBox.transmission->SetVoltage(frVoltage);
  _config.rearLeftGearBox.transmission->SetVoltage(rlVoltage);
  _config.rearRightGearBox.transmission->SetVoltage(rrVoltage);
}

void MecanumDrivebase::SetIdle(){
  _activeState = MecanumDrivebaseState::kIdle;
}
void MecanumDrivebase::SetManual(){
  _activeState = MecanumDrivebaseState::kManual;
}

void MecanumDrivebase::SetVelocity(frc::ChassisSpeeds speeds) {
  _activeState = MecanumDrivebaseState::kVelocity;
  _targetSpeeds = speeds;
}