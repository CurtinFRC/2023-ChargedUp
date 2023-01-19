#pragma once

#include "RobotMap.h"

#include <string>

#include <frc/TimedRobot.h>
#include <frc/event/EventLoop.h>

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

  void SimulationInit() override;
  void SimulationPeriodic() override;

 private:
  frc::EventLoop loop;
  
  RobotMap map;
  Armavator *armavator;
  wom::SwerveDrive *swerve;
  SideIntake *sideIntake;
};
