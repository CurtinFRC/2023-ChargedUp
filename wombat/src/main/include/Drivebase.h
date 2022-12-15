#pragma once 

#include "Gearbox.h"
#include <frc/SpeedController.h>
#include 


namespace wom {
  enum class DrivetrainState {
    kManual, 
    kPID,
    kIdle
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

    void Set(double leftPower, double rightPower);
    void SetVoltage(double left, double right);

    void SetInverted(bool inverted = false);
    // void GetInverted() { return _config.reversed; }

    // DrivetrainConfig &GetConfig() { return _config; }

    double GetLeftDistance();
    double GetRightDistance();

   protected: 
    Gearbox &GetLeft();
    Gearbox &GetRight();
   private: 
    DrivetrainConfig _config;
  };
};