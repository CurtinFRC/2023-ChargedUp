#pragma once 

#include "Gearbox.h"
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"
#include "VoltageController.h"
#include <frc/interfaces/Gyro.h>
#include "PID.h"

#include <units/angular_velocity.h>
#include <units/charge.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

namespace wom {
  enum class SwerveModuleState {
    kIdle, 
    kPID
  };

  struct SwerveModuleConfig {
    frc::Translation2d position;

    Gearbox driveMotor;
    Gearbox turnMotor;

    units::meter_t wheelRadius;
  };

  class SwerveModule {
   public:
    using angle_pid_conf_t = PIDConfig<units::radian, units::volt>;
    using velocity_pid_conf_t = PIDConfig<units::meters_per_second, units::volt>;

    SwerveModule(std::string path, SwerveModuleConfig config, angle_pid_conf_t anglePID, velocity_pid_conf_t velocityPID);
    void OnUpdate(units::second_t dt);

    void SetIdle();
    void SetPID(units::radian_t angle, units::meters_per_second_t speed);
  
    frc::SwerveModuleState GetState();

    units::meters_per_second_t GetSpeed() const;

    const SwerveModuleConfig &GetConfig() const;

   private:
    SwerveModuleConfig _config;
    SwerveModuleState _state;

    PIDController<units::radians, units::volt> _anglePIDController;
    PIDController<units::meters_per_second, units::volt> _velocityPIDController;
  };

  struct SwerveDriveConfig {
    SwerveModule::angle_pid_conf_t anglePID;
    SwerveModule::velocity_pid_conf_t velocityPID;

    wpi::array<SwerveModuleConfig, 4> modules;

    frc::Gyro *gyro;

    PIDConfig<units::radian, units::radians_per_second> poseAnglePID;
    PIDConfig<units::meter, units::meters_per_second> posePositionPID;

    wpi::array<double, 3> stateStdDevs{0.0, 0.0, 0.0};
    wpi::array<double, 1> localMeasurementStdDevs{0.0};
    wpi::array<double, 3> visionMeasurementStdDevs{0.0, 0.0, 0.0};

  };

  enum class SwerveDriveState {
    kIdle, 
    kVelocity,
    kFieldRelativeVelocity,
    kPose
  };

  struct FieldRelativeSpeeds {
    /**
     * Represents the velocity in the x dimension (your alliance to opposite alliance)
     */
    units::meters_per_second_t vx{0};
    /**
     * Represents the velocity in the y dimension (to your left when standing behind alliance wall)
     */
    units::meters_per_second_t vy{0};
    /**
     * The angular velocity of the robot (CCW is +)
     */
    units::radians_per_second_t omega{0};

    frc::ChassisSpeeds ToChassisSpeeds(const units::radian_t robotHeading);
  };

  class SwerveDrive : public behaviour::HasBehaviour {
   public:
    SwerveDrive(std::string path, SwerveDriveConfig config, frc::Pose2d initialPose);

    void OnUpdate(units::second_t dt);

    void SetIdle();
    void SetVelocity(frc::ChassisSpeeds speeds);
    void SetFieldRelativeVelocity(FieldRelativeSpeeds speeds);
    void SetPose(frc::Pose2d pose);
    bool IsAtSetPose();

    void ResetPose(frc::Pose2d pose);

    frc::Pose2d GetPose();
    void AddVisionMeasurement(frc::Pose2d pose, units::second_t timestamp);

    SwerveDriveConfig &GetConfig() { return _config; }

   protected:

   private:
    SwerveDriveConfig _config;
    SwerveDriveState _state = SwerveDriveState::kIdle;
    std::vector<SwerveModule> _modules;

    frc::ChassisSpeeds _target_speed;
    FieldRelativeSpeeds _target_fr_speeds;

    frc::SwerveDriveKinematics<4> _kinematics;
    frc::SwerveDrivePoseEstimator<4> _poseEstimator;

    PIDController<units::radian, units::radians_per_second> _anglePIDController;
    PIDController<units::meter, units::meters_per_second> _xPIDController;
    PIDController<units::meter, units::meters_per_second> _yPIDController;
  };
}

