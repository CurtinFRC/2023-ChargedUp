#pragma once

#include <string>
#include "RobotMap.h"
#include "Intake.h"

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

using namespace frc;

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
  RobotMap map;
  Intake *intake;
};
