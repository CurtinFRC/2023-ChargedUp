#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <iostream>
#include <ctre/Phoenix.h>

#include <frc/I2C.h>
#include <frc/SPI.h>
#include <frc/interfaces/Gyro.h>
#include "AHRS.h"

using namespace frc;


// #include <frc/RobotBase.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::XboxController *driver;
  TalonSRX *dropWheelOne, *dropWheelTwo;
  TalonFX *leftDrive, *rightDrive;

  // wml::sensors::NavX navX{frc::SPI::Port::kMXP, 200};
  // wml::sensors::NavXGyro gyro{navX.Angular(wml::sensors::AngularAxis::YAW)};

  // robotGyro gyro{SPI::Port::kMXP};
  AHRS *ahrs;

  bool dropToggle = false;
  bool normalDrive = true;
  double dropSpeed = 0;

  double dropPIDSpeed = 0;

  double targetAngle = 0;

  double kP = 0.0004;
  double kI = 0.000;
  double kD = 0;

  double _sum = 0;
  double _previousError = 0;
};
