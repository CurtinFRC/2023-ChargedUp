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

#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>

namespace wom {
  enum class MechanumState {
      kIdle,
      kRaw,
      kManual,
      kPose
  };

  struct MechanumConfig {
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
  };

  class MechanumBase : public behaviour::HasBehaviour {
    public :
      MechanumBase(MechanumConfig config);

      void OnUpdate(units::second_t dt);
      void OnStart();


      // config getter
      MechanumConfig *GetConfig() { return _config; }

      // Sets state
      void SetState(MechanumState state) { _state = state; }


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
      MechanumConfig *_config;
      MechanumState _state;

      // PID velocity controllers
      PIDController<units::meters_per_second, units::volt> topRightVelocityController;
      PIDController<units::meters_per_second, units::volt> topLeftVelocityController;
      PIDController<units::meters_per_second, units::volt> bottomRightVelocityController;
      PIDController<units::meters_per_second, units::volt> bottomLeftVelocityController;
  };
};

class MechanumTurnToAngle : public behaviour::Behaviour {
  public :
    MechanumTurnToAngle(MechanumBase driveBase, units::radian_t angle);

    void OnStart() override;
    void OnTick(units::second_t dt) override;

  private :
  MechanumBase *_drivebase;
};

class MechanumDriveToPose : public behaviour::Behaviour {
  public :
    MechanumDriveToPose(MechanumBase driveBase, frc::Pose2d pose);

    void OnStart() override;
    void OnTick(units::second_t dt) override;
    
  private :
    MechanumBase *_drivebase;
};