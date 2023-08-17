#pragma once

#include "RobotMap.h"
#include "Vision.h"
#include "WarpAuto.h"

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
#include "behaviour/IntakeBehaviour.h"
#include "Intake.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/event/BooleanEvent.h>
#include "frc/smartdashboard/SendableChooser.h"

#include <map>


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
  frc::EventLoop loop;
  RobotMap map;
  behaviour::BehaviourScheduler *sched;

  //creates nessesary instances to use in robot.cpp and robotmap.h
  wom::SwerveDrive *swerve;
  Armavator *armavator;
  Vision *vision;
  Gripper *gripper;
  Drivebase *drivebase = new Drivebase{swerve, gyro};
  //Intake *intake;
  
  wom::NavX *gyro;

  bool compressorToggle = false;
  bool intakeSol = false;
  bool gripperSol = false;

  units::meter_t _elevatorSetpoint = 0_m;
  units::radian_t _armSetpoint = 0_deg;

 // frc::SendableChooser<std::string> m_chooser;

 // std::string m_autoSelected;

 // const std::string kLowPlace = "kLowPlace";
 // const std::string kLowPlaceTaxi = "kLowPlaceTaxi";
 // const std::string kHighPlaceTaxi = "kHighPlaceTaxi";
 // const std::string kHighPlace = "kHighPlace";
 // const std::string kBalence = "kBalence";
 // const std::string kDock = "kDock";

  // std::string defaultAuto = "kLowPlace";
  // std::vector<std::string> autoOptions = {
  //   kLowPlace,
  //   kLowPlaceTaxi,
  //   kHighPlaceTaxi,
  //   kHighPlace,
  //   kDock
  // };

//  Auto warpAuto = Auto(drivebase, armavator, gripper);
};
