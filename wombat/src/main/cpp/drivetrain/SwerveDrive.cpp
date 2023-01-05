#include "drivetrain/SwerveDrive.h"

using namespace wom;

SwerveModule::SwerveModule(std::string path, SwerveModuleConfig config, SwerveModule::angle_pid_conf_t anglePID, SwerveModule::velocity_pid_conf_t velocityPID) 
  : _config(config), _anglePIDController(path + "/pid/angle", anglePID), _velocityPIDController(path + "/pid/velocity", velocityPID) {
    _anglePIDController.SetWrap(360_deg);
  }

void SwerveModule::OnUpdate(units::second_t dt) {
  units::volt_t driveVoltage{0};
  units::volt_t turnVoltage{0};

  switch(_state) {
    case SwerveModuleState::kIdle:
      driveVoltage = 0_V;
      turnVoltage = 0_V;
      break;
    case SwerveModuleState::kPID:
      {
        auto feedforward = _config.driveMotor.motor.Voltage(0_Nm, units::radians_per_second_t{(_velocityPIDController.GetSetpoint() / _config.wheelRadius).value()});
        driveVoltage = _velocityPIDController.Calculate(GetSpeed(), dt, feedforward);
        turnVoltage = _anglePIDController.Calculate(_config.turnMotor.encoder->GetEncoderPosition(), dt);
      }
      break;
  }

  _config.driveMotor.transmission->SetVoltage(driveVoltage);
  _config.turnMotor.transmission->SetVoltage(turnVoltage);
}

void SwerveModule::SetIdle() {
  _state = SwerveModuleState::kIdle;
}

void SwerveModule::SetPID(units::radian_t angle, units::meters_per_second_t speed) {
  _state = SwerveModuleState::kPID;
  _anglePIDController.SetSetpoint(angle);
  _velocityPIDController.SetSetpoint(speed);
}

units::meters_per_second_t SwerveModule::GetSpeed() const {
  return units::meters_per_second_t{_config.driveMotor.encoder->GetEncoderAngularVelocity().value() * _config.wheelRadius.value()};
}

frc::SwerveModuleState SwerveModule::GetState() {
  return frc::SwerveModuleState {
    GetSpeed(),
    _config.turnMotor.encoder->GetEncoderPosition()
  };
}

const SwerveModuleConfig &SwerveModule::GetConfig() const {
  return _config;
}

SwerveDrive::SwerveDrive(std::string path, SwerveDriveConfig config, frc::Pose2d initialPose) :
  _config(config),
  _kinematics( _config.modules[0].position, _config.modules[1].position, _config.modules[2].position, _config.modules[3].position),
  _poseEstimator(_config.gyro->GetRotation2d(), initialPose, _kinematics, _config.stateStdDevs, _config.localMeasurementStdDevs, _config.visionMeasurementStdDevs),
  _anglePIDController(path + "/pid/heading", _config.poseAnglePID), _xPIDController(path + "/pid/x", _config.posePositionPID), _yPIDController(path + "/pid/y", _config.posePositionPID) {

  _anglePIDController.SetWrap(360_deg);
  
  int i = 1;
  for (auto cfg : _config.modules) {
    _modules.emplace_back(path + "/modules/" + std::to_string(i), cfg, config.anglePID, config.velocityPID);
    i++;
  }
}

frc::ChassisSpeeds FieldRelativeSpeeds::ToChassisSpeeds(const units::radian_t robotHeading) {
  return frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx, vy, omega, frc::Rotation2d{robotHeading});
}

void SwerveDrive::OnUpdate(units::second_t dt) {
  switch (_state) {
    case SwerveDriveState::kIdle:
      for (auto mod = _modules.begin(); mod < _modules.end(); mod++) {
        mod->SetIdle();
      }
      break;
    case SwerveDriveState::kPose:
      {
        _target_fr_speeds.vx = _xPIDController.Calculate(GetPose().X(), dt);
        _target_fr_speeds.vy = _yPIDController.Calculate(GetPose().Y(), dt);
        _target_fr_speeds.omega = _anglePIDController.Calculate(GetPose().Rotation().Radians(), dt);
      }
    case SwerveDriveState::kFieldRelativeVelocity:
      _target_speed = _target_fr_speeds.ToChassisSpeeds(GetPose().Rotation().Radians());
    case SwerveDriveState::kVelocity:
      {
        auto target_states = _kinematics.ToSwerveModuleStates(_target_speed);
        for (int i = 0; i < _modules.size(); i++) {
          _modules[i].SetPID(target_states[i].angle.Radians(), target_states[i].speed);
        }
      }
      break;

  }

  for (auto mod = _modules.begin(); mod < _modules.end(); mod++) {
    mod->OnUpdate(dt);
  }

  _poseEstimator.Update(_config.gyro->GetRotation2d(), _modules[0].GetState(), _modules[1].GetState(), _modules[2].GetState(), _modules[3].GetState());
}

void SwerveDrive::SetIdle() {
  _state = SwerveDriveState::kIdle;
}

void SwerveDrive::SetVelocity(frc::ChassisSpeeds speeds) {
  _state = SwerveDriveState::kVelocity;
  _target_speed = speeds;
}

void SwerveDrive::SetFieldRelativeVelocity(FieldRelativeSpeeds speeds) {
  _state = SwerveDriveState::kFieldRelativeVelocity;
  _target_fr_speeds = speeds;
}

void SwerveDrive::SetPose(frc::Pose2d pose) {
  _state = SwerveDriveState::kPose;
  _anglePIDController.SetSetpoint(pose.Rotation().Radians());
  _xPIDController.SetSetpoint(pose.X());
  _yPIDController.SetSetpoint(pose.Y());
}

bool SwerveDrive::IsAtSetPose() {
  return _anglePIDController.IsStable() && _xPIDController.IsStable() && _yPIDController.IsStable();
}

void SwerveDrive::ResetPose(frc::Pose2d pose) {
  _poseEstimator.ResetPosition(pose, _config.gyro->GetRotation2d());
}

frc::Pose2d SwerveDrive::GetPose() {
  return _poseEstimator.GetEstimatedPosition();
}

void SwerveDrive::AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp) {
  _poseEstimator.AddVisionMeasurement(pose, timestamp);
}
