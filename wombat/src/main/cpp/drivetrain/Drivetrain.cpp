#include "drivetrain/Drivetrain.h"

using namespace wom;

Drivetrain::Drivetrain(DrivetrainConfig config) : _config(config), _kinematics(config.trackWidth), _leftVelocityController(config.velocityPID), _rightVelocityController(config.velocityPID) {}

void Drivetrain::OnUpdate(units::second_t dt) {
  units::volt_t leftVoltage{0};
  units::volt_t rightVoltage{0};
  auto wheelSpeeds = _kinematics.ToWheelSpeeds(_speed);
  wheelSpeeds.Desaturate((_config.leftDrive.motor.freeSpeed.value() * _config.wheelRadius.value()) * 1_mps);

  switch (_state) {
    case DrivetrainState::kManual:
      leftVoltage = _leftManualSetpoint;
      rightVoltage = _rightManualSetpoint;
      break;
    case DrivetrainState::kIdle:
      leftVoltage = 0_V;
      rightVoltage = 0_V;
      break;
    case DrivetrainState::kRaw:
      leftVoltage = _leftRawSetpoint;
      rightVoltage = _rightRawSetpoint;
      break;
    case DrivetrainState::kVelocity:
      {
        _leftVelocityController.SetSetpoint(wheelSpeeds.left);
        _rightVelocityController.SetSetpoint(wheelSpeeds.right);
        auto feedforwardLeft = _config.leftDrive.motor.Voltage(0_Nm, units::radians_per_second_t{(_leftVelocityController.GetSetpoint() / _config.wheelRadius).value()});
        auto feedforwardRight = _config.rightDrive.motor.Voltage(0_Nm, units::radians_per_second_t{(_rightVelocityController.GetSetpoint() / _config.wheelRadius).value()});
        leftVoltage = _leftVelocityController.Calculate(GetLeftSpeed(), dt, feedforwardLeft);
        rightVoltage = _rightVelocityController.Calculate(GetRightSpeed(), dt, feedforwardRight);
      }
      break;
    case DrivetrainState::kPose:

      break;
  }


  _config.leftDrive.transmission->SetVoltage(leftVoltage);
  _config.rightDrive.transmission->SetVoltage(rightVoltage);

  // units::newton_meter_t max_torque_at_current_limit = _config.leftDrive.motor.Torque(_config.currentLimit);
  // units::volt_t max_voltage_for_current_limit = _config.leftDrive.motor.Voltage(max_torque_at_current_limit, wheelSpeeds.left);

  // leftVoltage = 1_V * std::min(voltage.value(), max_voltage_for_current_limit.value());

}

void Drivetrain::SetRawVoltage(units::volt_t left, units::volt_t right) {
  _state = DrivetrainState::kRaw;
  _leftRawSetpoint = left;
  _rightRawSetpoint = right;
}

void Drivetrain::SetManual(double leftPower, double rightPower) {
  _state = DrivetrainState::kManual;

  _leftRawSetpoint = leftPower * 12_V;
  _rightRawSetpoint = rightPower * 12_V;
}

void Drivetrain::SetIdle() {
  _state = DrivetrainState::kIdle;
}

void Drivetrain::SetVelocity(frc::ChassisSpeeds speeds) {
  _state = DrivetrainState::kVelocity;
  _speed = speeds;
}

void Drivetrain::SetTargetPose(frc::Pose2d pose) {
  _state = DrivetrainState::kPose;
  _targetPose = pose;
}

units::meter_t Drivetrain::GetLeftDistance() const {
  return _config.leftDrive.encoder->GetEncoderPosition().value() * _config.wheelRadius;
}

units::meter_t Drivetrain::GetRightDistance() const {
  return _config.rightDrive.encoder->GetEncoderPosition().value() * _config.wheelRadius;
}

units::meters_per_second_t Drivetrain::GetLeftSpeed() const {
  return units::meters_per_second_t{_config.leftDrive.encoder->GetEncoderAngularVelocity().value() * _config.wheelRadius.value()};
}

units::meters_per_second_t Drivetrain::GetRightSpeed() const {
  return units::meters_per_second_t{_config.rightDrive.encoder->GetEncoderAngularVelocity().value() * _config.wheelRadius.value()};
}

// Drivetrain Behaviours

DrivetrainDriveDistance::DrivetrainDriveDistance(Drivetrain *d, units::meter_t length, std::optional<units::meter_t> radius) : _drivetrain(d), _pid(d->GetConfig().distancePID), _radius(radius) {
  Controls(d);
  _pid.SetSetpoint(length);
}

units::meter_t DrivetrainDriveDistance::GetDistance() const {
  return (_drivetrain->GetLeftDistance() + _drivetrain->GetRightDistance()) / 2;
}

void DrivetrainDriveDistance::OnStart() {
  _start_distance = GetDistance();
}

void DrivetrainDriveDistance::OnTick(units::second_t dt) {
  auto speed = _pid.Calculate(GetDistance() - _start_distance, dt);
  if (_radius.has_value()) {
    _drivetrain->SetVelocity(frc::ChassisSpeeds {
      speed, 0_mps, units::radians_per_second_t{(speed / _radius.value()).value()}
    });
  } else {
    _drivetrain->SetVelocity(frc::ChassisSpeeds {
      speed, 0_mps, 0_deg_per_s
    });
  }

  if (_pid.IsStable()) {
    _drivetrain->SetIdle();
    SetDone();
  }
}

// Turn to angle behaviours 

DrivetrainTurnToAngle::DrivetrainTurnToAngle(Drivetrain *d, units::degree_t setpoint) : _drivetrain(d), _pid(d->GetConfig().anglePID) {
  Controls(d);
  _pid.SetSetpoint(setpoint);
} 

units::degree_t DrivetrainTurnToAngle::GetAngle() const {
  return _drivetrain->GetConfig().gyro->GetRotation2d().Degrees();
}

void DrivetrainTurnToAngle::OnStart() {
  _start_angle = GetAngle();
}

void DrivetrainTurnToAngle::OnTick(units::second_t dt) {
  auto speed = _pid.Calculate(GetAngle() - _start_angle, dt);
  _drivetrain->SetVelocity(frc::ChassisSpeeds {
    0_mps, 0_mps, speed
  });

  if (_pid.IsStable()) {
    _drivetrain->SetIdle();
    SetDone();
  }
}