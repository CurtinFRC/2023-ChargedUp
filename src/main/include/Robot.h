#pragma once

#include "RobotMap.h"
#include "Vision.h"

#include <string>
#include <iostream>

#include <ctre/Phoenix.h>
#include <frc/TimedRobot.h>
#include <frc/event/EventLoop.h>
#include "ControlUtil.h"
#include <units/math.h>

#include <stdio.h>

#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/event/BooleanEvent.h>

using namespace frc;

//overrides the preset robot functions so that we can edit them
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

  // void SimulationInit() override;
  // void SimulationPeriodic() override;

 private:
  frc::EventLoop loop;
  
  //creates nessesary instances to use in robot.cpp and robotmap.h
  RobotMap map;
  Armavator *armavator;
  wom::SwerveDrive *swerve;
  bool intakeSol = false;
  bool gripperSol = false;
  Vision *vision;
  //SwerveModuleTest *swerveModule;
  SideIntake *sideIntake;
  Gripper *gripper;

  units::meter_t _elevatorSetpoint = 0_m;
  units::radian_t _armSetpoint = 0_deg;
};
