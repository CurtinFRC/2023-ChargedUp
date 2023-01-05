#include "DCMotor.h"
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <frc/RobotController.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

class SwerveModuleSim {
 public:
  SwerveModuleSim(wom::DCMotor turnMotor, wom::DCMotor driveMotor, units::kilogram_t mass, units::kilogram_square_meter_t J, units::meter_t wheel_radius)
    : _turnMotor(turnMotor), _driveMotor(driveMotor), _J(J), _effective_mass(mass), _wheel_radius(wheel_radius) {}
 
  frc::SwerveModuleState Calculate(units::volt_t turnVoltage, units::volt_t driveVoltage, units::second_t dt) {
    auto vbat = frc::RobotController::GetBatteryVoltage();
    turnVoltage = units::math::min(units::math::max(turnVoltage, -vbat), vbat);
    driveVoltage = units::math::min(units::math::max(driveVoltage, -vbat), vbat);

    // Turn
    _turn_amps = _turnMotor.Current(_angle_speed, turnVoltage);
    auto turn_torque = _turnMotor.Torque(_turn_amps);
    auto angular_accel = units::radians_per_second_squared_t{(turn_torque / _J).value()};
    _angle_speed += angular_accel * dt;
    _angle += _angle_speed * dt;

    // Drive
    _drive_amps = _driveMotor.Current(_drive_speed, driveVoltage);
    auto drive_torque = _driveMotor.Torque(_drive_amps);
    auto acceleration = drive_torque / (_effective_mass * _wheel_radius);
    _velocity += acceleration * dt;
    _position += _velocity * dt;
    _drive_speed = units::radians_per_second_t{(_velocity / _wheel_radius).value()};

    return GetState();
  }

  units::radian_t GetAngle() const {
    return _angle;
  }

  units::meter_t GetPosition() const {
    return _position;
  }
  
  units::meters_per_second_t GetVelocity() const {
    return _velocity;
  }

  units::radians_per_second_t GetSpeed() const {
    return _drive_speed;
  }

  frc::SwerveModuleState GetState() const {
    return frc::SwerveModuleState {
      GetVelocity(),
      frc::Rotation2d{GetAngle()}
    };
  }

 private:
  wom::DCMotor _turnMotor;
  wom::DCMotor _driveMotor;
  units::kilogram_square_meter_t _J;
  units::kilogram_t _effective_mass;
  units::meter_t _wheel_radius;
  
  units::ampere_t _turn_amps{0};
  units::ampere_t _drive_amps{0};

  units::radian_t _angle{0};
  units::radians_per_second_t _angle_speed{0};

  units::meter_t _position{0};
  units::meters_per_second_t _velocity{0};
  units::radians_per_second_t _drive_speed{0};
};

class SwerveSim {
 public:
  SwerveSim(frc::SwerveDriveKinematics<4> kinematics, wpi::array<SwerveModuleSim*, 4> mods) : _kinematics(kinematics), modules(mods) { }
 
  void Calculate(wpi::array<units::volt_t, 4> turnVoltage, wpi::array<units::volt_t, 4> driveVoltage, units::second_t dt) {
    frc::SwerveModuleState  s0 = modules[0]->Calculate(turnVoltage[0], driveVoltage[0], dt),
                            s1 = modules[1]->Calculate(turnVoltage[1], driveVoltage[1], dt),
                            s2 = modules[2]->Calculate(turnVoltage[2], driveVoltage[2], dt),
                            s3 = modules[3]->Calculate(turnVoltage[3], driveVoltage[3], dt);

    auto chassis_state = _kinematics.ToChassisSpeeds(s0, s1, s2, s3);
    x += (chassis_state.vx * units::math::cos(-theta) + chassis_state.vy * units::math::sin(-theta)) * dt;
    y += (chassis_state.vx * -units::math::sin(-theta) + chassis_state.vy * units::math::cos(-theta)) * dt;
    theta += chassis_state.omega * dt;
  }

  units::meter_t x, y;
  units::radian_t theta;
  wpi::array<SwerveModuleSim*, 4> modules;
 private:
  frc::SwerveDriveKinematics<4> _kinematics;
};