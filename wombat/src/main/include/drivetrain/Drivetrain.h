#pragma once 

#include "Gearbox.h"
#include <frc/SpeedController.h>
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include "PID.h"

#include <units/angular_velocity.h>
#include <units/charge.h>

namespace wom {
  enum class DrivetrainState {
    kManual, 
    kIdle, 
    kRaw,
    kVelocity,
    kPose
  };

  struct DrivetrainConfig {
    Gearbox &leftDrive;
    Gearbox &rightDrive;

    frc::Gyro *gyro;

    units::meter_t wheelRadius;
    units::meter_t trackWidth;

    PIDConfig<units::meters_per_second, units::volt> pidConfig;
  };

  class Drivetrain : public behaviour::HasBehaviour {
   public: 
    Drivetrain(DrivetrainConfig config); 

    void OnUpdate(units::second_t dt);

    void SetRawVoltage(units::volt_t left, units::volt_t right);
    void SetManual(double leftPower, double rightPower);
    void SetIdle();
    void SetVelocity(frc::ChassisSpeeds speeds);
    void SetTargetPose(frc::Pose2d pose);

    DrivetrainConfig &GetConfig() { return _config; }

    units::meter_t GetLeftDistance() const;
    units::meter_t GetRightDistance() const;

    units::meters_per_second_t GetLeftSpeed() const;
    units::meters_per_second_t GetRightSpeed() const;

   protected: 
    Gearbox &GetLeft();
    Gearbox &GetRight();

   private: 
    DrivetrainConfig _config;
    DrivetrainState _state;

    units::volt_t _leftRawSetpoint;
    units::volt_t _rightRawSetpoint;

    units::volt_t _leftManualSetpoint;
    units::volt_t _rightManualSetpoint;

    frc::ChassisSpeeds _speed;
    frc::Pose2d _targetPose;

    frc::DifferentialDriveKinematics _kinematics;
    PIDController<units::meters_per_second, units::volt> _leftVelocityController;
    PIDController<units::meters_per_second, units::volt> _rightVelocityController;

  };

  class DrivetrainDriveDistance : public behaviour::Behaviour {
   public:
    using pid_config_t = PIDConfig<units::meter, units::meters_per_second>;

    DrivetrainDriveDistance(Drivetrain *d, pid_config_t pid, units::meter_t setpoint);

    units::meter_t GetDistance() const;

    void OnStart() override;
    void OnTick(units::second_t dt) override;
   private:
    Drivetrain *_drivetrain;
    units::meter_t _start_distance{0};

    PIDController<units::meter, units::meters_per_second> _pid;
  };
}

