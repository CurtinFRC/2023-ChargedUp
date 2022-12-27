#include "drivetrain/SwerveDrive.h"

using namespace wom;

SwerveModule::SwerveModule(SwerveModuleConfig config, SwerveModule::angle_pid_conf_t anglePID, SwerveModule::velocity_pid_conf_t velocityPID) 
  : _config(config), _anglePIDController(anglePID), _velocityPIDController(velocityPID) {
    _anglePIDController.SetWrap(180_deg);
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
        std::cout << _config.turnMotor.encoder->GetEncoderPosition().value() << std::endl;
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

const SwerveModuleConfig &SwerveModule::GetConfig() const {
  return _config;
}

SwerveDrive::SwerveDrive(SwerveDriveConfig config) :
  _config(config),
  _kinematics( _config.modules[0].position, _config.modules[1].position, _config.modules[2].position, _config.modules[3].position) {
  
  for (auto cfg : _config.modules) {
    _modules.emplace_back(cfg, config.anglePID, config.velocityPID);
  }
}

void SwerveDrive::OnUpdate(units::second_t dt) {
  auto chassis_states = frc::ChassisSpeeds::FromFieldRelativeSpeeds(_target_speed.vx, _target_speed.vy, _target_speed.omega, _config.gyro->GetRotation2d());
  auto target_module_states = _kinematics.ToSwerveModuleStates(chassis_states);

  switch (_state) {
    case SwerveDriveState::kIdle:
      for (auto i = 0; i < _modules.size(); i++) {
        _modules[i].SetIdle();
      }
      break;
    case SwerveDriveState::kVelocity:
      for (auto i = 0; i < _modules.size(); i++) {
        auto target_state = target_module_states[i];
        _modules[i].SetPID(target_state.angle.Radians(), target_state.speed);
      }
      break;
  }

  for (auto i = 0; i < _modules.size(); i++) {
    _modules[i].OnUpdate(dt);
  }
}

void SwerveDrive::SetIdle() {
  _state = SwerveDriveState::kIdle;
}

void SwerveDrive::SetVelocity(frc::ChassisSpeeds speeds) {
  _state = SwerveDriveState::kVelocity;
  _target_speed = speeds;
}

