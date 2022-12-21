#pragma once 

#include "Gearbox.h"
#include <frc/SpeedController.h>
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"
#include <frc/interfaces/Gyro.h>

#include <units/charge.h>

namespace wom {
  enum class DrivetrainState {
    kManual, 
    kPID,
    kIdle,
    kTurnToAngle,
    kDriveToDistance, 
    kSpline
  };

  struct DrivetrainConfig {
    Gearbox &leftDrive;
    Gearbox &rightDrive;

    frc::Gyro *gyro;

    double wheelRadius;
    bool reversed = false;
  };

  class Drivetrain : public behaviour::HasBehaviour {
   public: 
    Drivetrain(DrivetrainConfig config); 

    void OnUpdate(units::second_t dt);

    void Set(double leftPower, double rightPower);
    void SetVoltage(double left, double right);

    void SetInverted(bool inverted = false);
    bool GetInverted() { return _config.reversed; }

    DrivetrainConfig &GetConfig() { return _config; }

    double GetLeftDistance();
    double GetRightDistance();

    void TurnToAngle(double goal, units::second_t dt);

   protected: 
    Gearbox &GetLeft();
    Gearbox &GetRight();
   private: 
    DrivetrainConfig _config;
  };

  struct SwerveDriveConfig {

  };

  class SwerveDrive : public behaviour::HasBehaviour {
   public:
    SwerveDrive(SwerveDriveConfig config);
   protected:

   private:
    SwerveDriveConfig _config;
  };

  struct WaspDriveConfig {
    
  };

  class WaspDrive : public behaviour::HasBehaviour {
   public: 
    WaspDrive(WaspDriveConfig config);

   private: 
    WaspDriveConfig _config;
  };
}

