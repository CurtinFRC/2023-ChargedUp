#pragma once

#include "PID.h"
#include "Gearbox.h"
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"

#include <units/angular_velocity.h>
#include <units/charge.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <frc/XboxController.h>

#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>

namespace wom {
  enum class MecanumState {
      kIdle,
      kManual,
      kPose,
      kZeroing
  };

  class Vector {
    public :
      struct WheelVector {
        double X;
        double Y;
      };

      struct MecanumVector {
        WheelVector topLeft;
        WheelVector topRight;
        WheelVector bottomLeft;
        WheelVector bottomRight;

        WheelVector direction = {(topLeft.X + topRight.X + bottomLeft.X + bottomRight.X), (topLeft.Y + topRight.Y + bottomLeft.Y + bottomRight.Y)};
    };
  };

  struct MecanumConfig {
    using pose_position_conf_t = PIDConfig<units::meter, units::meters_per_second>;
    using pose_angle_conf_t = PIDConfig<units::radian, units::radians_per_second>;

    Gearbox &topLeft;
    Gearbox &topRight;
    Gearbox &bottomLeft;

    Gearbox &bottomRight;

    frc::Gyro *gyro;

    units::meter_t wheelRadius;
    units::meter_t wheelWidth;

    PIDConfig<units::meters_per_second, units::volt> velocityPID;
    PIDConfig<units::meter, units::meters_per_second> distancePID;
    PIDConfig<units::degree, units::degrees_per_second> anglePID;

    pose_position_conf_t posePositionPID;
    pose_angle_conf_t poseAnglePID;


  };

  class MecanumBase : public behaviour::HasBehaviour {
    public :
      MecanumBase(MecanumConfig *config, std::string path, Vector::MecanumVector direction, frc::XboxController driver);

      void OnUpdate(units::second_t dt);
      void OnStart();


      // config getter
      MecanumConfig *GetConfig() { return _config; }

      // Sets state
      void SetState(MecanumState state) { _state = state; }

      void SetPose(frc::Pose2d pose);


      // getters
      units::meter_t GetLeftDistance() const;
      units::meter_t GetRightDistance() const;

      units::meters_per_second_t GetLeftSpeed() const;
      units::meters_per_second_t GetRightSpeed() const;

      units::meter_t GetDistance();
      units::radian_t GetAngle();


    protected :
    // gearbox getters
      Gearbox GetTopLeft() { return _config->topLeft; }
      Gearbox GetTopRight() { return _config->topRight; }
      Gearbox GetBottomLeft() { return _config->bottomLeft; }
      Gearbox GetBottomRight() { return _config->bottomRight; }

    private :
      MecanumConfig *_config;
      MecanumState _state;
      Vector::MecanumVector _direction;
      frc::XboxController &_driver;
      units::volt_t voltage;

      // PID velocity controllers
      PIDController<units::meters_per_second, units::volt> topRightVelocityController;
      PIDController<units::meters_per_second, units::volt> topLeftVelocityController;
      PIDController<units::meters_per_second, units::volt> bottomRightVelocityController;
      PIDController<units::meters_per_second, units::volt> bottomLeftVelocityController;

      PIDController<units::meter, units::meters_per_second> _xPIDController;
      PIDController<units::meter, units::meters_per_second> _yPIDController;
      PIDController<units::radian, units::radians_per_second> _anglePIDController;
  };

class MecanumTurnToAngle : public behaviour::Behaviour {
  public :
    MecanumTurnToAngle(MecanumBase driveBase, units::radian_t angle);

    void OnStart() override;
    void OnTick(units::second_t dt) override;

  private :
  MecanumBase *_drivebase;
};

class MecanumDriveToPose : public behaviour::Behaviour {
  public :
    MecanumDriveToPose(MecanumBase driveBase, frc::Pose2d pose);

    void OnStart() override;
    void OnTick(units::second_t dt) override;
    
  private :
    MecanumBase *_drivebase;
};
};