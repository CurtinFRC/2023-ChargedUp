#pragma once 

#include "Gearbox.h"
#include <frc/SpeedController.h>
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"
#include "VoltageController.h"
#include <frc/interfaces/Gyro.h>
#include "PID.h"

#include <units/angular_velocity.h>
#include <units/charge.h>

#include <frc/kinematics/SwerveDriveKinematics.h>

namespace wom {
  enum class SwerveModuleState {
    kIdle, 
    kPID
  };

  struct SwerveModuleConfig {
    frc::Translation2d position;

    Gearbox &driveMotor;
    Gearbox &turnMotor;

    units::meter_t wheelRadius;
  };

  class SwerveModule {
   public:
    using angle_pid_conf_t = PIDConfig<units::radians, units::volt>;
    using velocity_pid_conf_t = PIDConfig<units::meters_per_second, units::volt>;

    SwerveModule(SwerveModuleConfig config, angle_pid_conf_t anglePID, velocity_pid_conf_t velocityPID);

    void OnUpdate(units::second_t dt);

    void SetIdle();
    void SetPID(units::radian_t angle, units::meters_per_second_t speed);
  
    units::meters_per_second_t GetSpeed() const;

   private:
    SwerveModuleConfig _config;
    SwerveModuleState _state;

    PIDController<units::radians, units::volt> _anglePIDController;
    PIDController<units::meters_per_second, units::volt> _velocityPIDController;
  };

  struct SwerveDriveConfig {
    PIDConfig<units::degree, units::degrees_per_second> anglePID;
    PIDConfig<units::meters_per_second, units::volt> velocityPID;

    wpi::array<SwerveModuleConfig, 4> &modules;

    frc::Gyro *gyro;
  };

  enum class SwerveDriveState {
    kIdle, 
    kVelocity
  };

  class SwerveDrive : public behaviour::HasBehaviour {
   public:
    SwerveDrive(SwerveDriveConfig config);

    void OnUpdate(units::second_t dt);

    void SetIdle();
    void SetVelocity();

    SwerveDriveConfig &GetConfig() { return _config; }

   protected:

   private:
    SwerveDriveConfig _config;
    SwerveDriveState _state;

    frc::SwerveDriveKinematics<4> _kinematics;
  };
}

