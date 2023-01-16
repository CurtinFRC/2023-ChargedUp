#pragma once 

#include "Gearbox.h"
#include "Gyro.h"
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"
#include "VoltageController.h"
#include <frc/interfaces/Gyro.h>
#include "PID.h"

#include <units/angular_velocity.h>
#include <units/charge.h>
#include <units/moment_of_inertia.h>

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

    void WriteNT(std::shared_ptr<nt::NetworkTable> table) const;
  };

  class SwerveModule {
   public:
    using angle_pid_conf_t = PIDConfig<units::radian, units::volt>;
    using velocity_pid_conf_t = PIDConfig<units::meters_per_second, units::volt>;

    SwerveModule(std::string path, SwerveModuleConfig config, angle_pid_conf_t anglePID, velocity_pid_conf_t velocityPID);
    void OnUpdate(units::second_t dt);

    void SetIdle();
    void SetPID(units::radian_t angle, units::meters_per_second_t speed);
  
    // frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition() const;

    units::meters_per_second_t GetSpeed() const;
    units::meter_t GetDistance() const;

    const SwerveModuleConfig &GetConfig() const;

   private:
    SwerveModuleConfig _config;
    SwerveModuleState _state;

    PIDController<units::radians, units::volt> _anglePIDController;
    PIDController<units::meters_per_second, units::volt> _velocityPIDController;

    std::shared_ptr<nt::NetworkTable> _table;
  };

  struct SwerveDriveConfig {
    using pose_angle_conf_t = PIDConfig<units::radian, units::radians_per_second>;
    using pose_position_conf_t = PIDConfig<units::meter, units::meters_per_second>;

    std::string path;
    SwerveModule::angle_pid_conf_t anglePID;
    SwerveModule::velocity_pid_conf_t velocityPID;

    wpi::array<SwerveModuleConfig, 4> modules;

    wom::Gyro *gyro;

    pose_angle_conf_t poseAnglePID;
    pose_position_conf_t posePositionPID;

    units::kilogram_t mass;

    wpi::array<double, 3> stateStdDevs{0.0, 0.0, 0.0};
    wpi::array<double, 3> visionMeasurementStdDevs{0.0, 0.0, 0.0};

    void WriteNT(std::shared_ptr<nt::NetworkTable> table);
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
    SwerveDrive(SwerveDriveConfig config, frc::Pose2d initialPose);

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

    std::shared_ptr<nt::NetworkTable> _table;
  };

  namespace sim {
    class SwerveDriveSim {
     public:
      SwerveDriveSim(SwerveDriveConfig config, units::kilogram_square_meter_t moduleJ);

      void Update(units::second_t dt);

      SwerveDriveConfig config;
      frc::SwerveDriveKinematics<4> kinematics;
      units::kilogram_square_meter_t moduleJ;
      std::shared_ptr<nt::NetworkTable> table;

      std::vector<std::shared_ptr<SimCapableEncoder>> driveEncoders;
      std::vector<std::shared_ptr<SimCapableEncoder>> turnEncoders;
      std::shared_ptr<SimCapableGyro> gyro;

      units::ampere_t totalCurrent = 0_A;
      wpi::array<units::ampere_t, 4> driveCurrents { 0_A, 0_A, 0_A, 0_A };
      wpi::array<units::ampere_t, 4> turnCurrents { 0_A, 0_A, 0_A, 0_A };
      wpi::array<units::radians_per_second_t, 4> turnSpeeds { 0_rad / 1_s, 0_rad / 1_s, 0_rad / 1_s, 0_rad / 1_s };
      wpi::array<units::radians_per_second_t, 4> driveSpeeds { 0_rad / 1_s, 0_rad / 1_s, 0_rad / 1_s, 0_rad / 1_s };
      wpi::array<units::meters_per_second_t, 4> driveVelocity { 0_mps, 0_mps, 0_mps, 0_mps };
      wpi::array<units::radian_t, 4> turnAngles { 0_rad, 0_rad, 0_rad, 0_rad };
      wpi::array<units::radian_t, 4> driveEncoderAngles { 0_rad, 0_rad, 0_rad, 0_rad };

      units::radian_t angle{0};
      units::radians_per_second_t angularVelocity{0};
      units::meter_t x{0}, y{0};
      units::meters_per_second_t vx{0}, vy{0};
    };
  }
}

