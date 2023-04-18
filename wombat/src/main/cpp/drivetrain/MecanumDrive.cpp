#include "drivetrain/MecanumDrive.hpp"
#include "ControlUtil.h"

using namespace wom;

MecanumBase::MecanumBase(MecanumConfig *config, std::string path, Vector::MecanumVector direction, frc::XboxController driver) : 
_config(config), 
topLeftVelocityController(path + "/pid/top/left", config->velocityPID), 
topRightVelocityController(path + "/pid/top/left", config->velocityPID), 
bottomLeftVelocityController(path + "/pid/bottom/left", config->velocityPID), 
bottomRightVelocityController(path + "/pid/bottom/right", config->velocityPID),
_direction(direction),
_driver(driver),
_xPIDController(path + "/pid/x", _config->posePositionPID),
_yPIDController(path + "pid/y", _config->posePositionPID),
_anglePIDController(path + "pid/angle", _config->poseAnglePID)
{}

void MecanumBase::OnStart() {
  std::cout << "starting" << std::endl;
  // zero encoders
}

void MecanumBase::SetPose(frc::Pose2d pose) {
  _anglePIDController.SetSetpoint(pose.Rotation().Radians());
  _xPIDController.SetSetpoint(pose.X());
  _yPIDController.SetSetpoint(pose.Y());
}

void MecanumBase::OnUpdate(units::second_t dt) {
  double transX = _driver.GetLeftY();
  double transY = _driver.GetLeftX();

  double rotate = _driver.GetRightX();

  //wheel speed math
  double den = units::math::max(abs(transX)+abs(transY)+abs(rotate), 1.0);

  double frontL = (transY - transX - rotate)/den;
  double backR = (transY + transX + rotate)/den;

  double backL = (-transY + transX - rotate)/den;
  double frontR = (transY + transX - rotate)/den;

  frc::Translation2d translation = {999_m, 999_m};
  frc::Rotation2d rotation = 9999_rad;
  frc::Pose2d randomSetpoint = {translation, rotation};
  switch (_state) {
    case MecanumState::kManual:


        _config->bottomLeft.transmission->SetVoltage(voltage * backL);
        _config->bottomRight.transmission->SetVoltage(voltage * backR);
        _config->topLeft.transmission->SetVoltage(voltage * frontL);
        _config->topRight.transmission->SetVoltage(voltage * frontR);
       break;
    case MecanumState::kIdle:
      break;
    case MecanumState::kPose:
      SetPose(randomSetpoint);
      break;
    case MecanumState::kZeroing:
      _config->bottomRight.encoder->ZeroEncoder();
      _config->bottomLeft.encoder->ZeroEncoder();
      _config->topRight.encoder->ZeroEncoder();
      _config->topLeft.encoder->ZeroEncoder();
      break;
  }
}

MecanumTurnToAngle::MecanumTurnToAngle(MecanumBase *driveBase, units::radian_t angle) : _drivebase(driveBase), _angle(angle) {}

void MecanumTurnToAngle::OnStart() {
  std::cout << "mecanum turning started" << std::endl;
  _drivebase->GetConfig()->bottomRight.encoder->ZeroEncoder();
  _drivebase->GetConfig()->bottomLeft.encoder->ZeroEncoder();
  _drivebase->GetConfig()->topRight.encoder->ZeroEncoder();
  _drivebase->GetConfig()->topLeft.encoder->ZeroEncoder();
}

void MecanumTurnToAngle::OnTick(units::second_t dt) {
  auto out = _drivebase->_anglePIDController.Calculate(_angle, dt);
  units::volt_t voltage = 12_V;
  while (out != 0_rad / 1_s) {
      _config->bottomLeft.transmission->SetVoltage(voltage);
      _config->topLeft.transmission->SetVoltage(voltage);
      _config->bottomRight.transmission->SetVoltage(-voltage);
      _config->topRight.transmission->SetVoltage(-voltage);
  }
}

MecanumDriveToPose::MecanumDriveToPose(MecanumBase *driveBase, frc::Pose2d pose) : _drivebase(driveBase), _pose(pose) {}

void MecanumDriveToPose::OnStart() {
  std::cout << "mecanum go to pose started" << std::endl;
  _drivebase->GetConfig()->bottomRight.encoder->ZeroEncoder();
  _drivebase->GetConfig()->bottomLeft.encoder->ZeroEncoder();
  _drivebase->GetConfig()->topRight.encoder->ZeroEncoder();
  _drivebase->GetConfig()->topLeft.encoder->ZeroEncoder();
}

void MecanumDriveToPose::OnTick(units::second_t dt) {
  auto outX = _drivebase->_xPIDController.Calculate(_pose.X(), dt);
  auto outY = _drivebase->_yPIDController.Calculate(_pose.Y(), dt);
  units::volt_t voltage = 12_V;

  while (outX != 0_m / 1_s && outY != 0_m / 1_s) {
       _config->bottomLeft.transmission->SetVoltage(voltage);
      _config->topLeft.transmission->SetVoltage(voltage);
      _config->bottomRight.transmission->SetVoltage(voltage);
      _config->topRight.transmission->SetVoltage(voltage);
  }
}