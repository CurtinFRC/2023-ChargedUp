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
  //Intake *intake;
  

  bool compressorToggle = false;
  bool intakeSol = false;
  bool gripperSol = false;

  units::meter_t _elevatorSetpoint = 0_m;
  units::radian_t _armSetpoint = 0_deg;

  frc::SendableChooser<std::string> m_chooser;


  std::string m_autoSelected;

  const std::string kTaxi = "kTaxi";
  const std::string HighPlace = "HighPlace";
  const std::string kBalance = "kBalance";
  const std::string kMidPlace = "kMidPlace";
  const std::string kBalanceAndHighPlace = "kBalanceAndHighPlace";
  const std::string kBalanceAndMidPlace = "kBalanceAndMidPlace";
  const std::string kSmallDrivebaseMovement = "kSmallDrivebaseMovement";
  const std::string kSmallArmMovement = "kSmallArmMovement";


    // Please modify the code below if auto is being changed, or new autos are added
  std::string defaultAuto = "kTaxi";
  std::vector<std::string> autoOptions = {
    kTaxi,
    HighPlace,
    kBalance,
    kMidPlace,
    kMidPlace,
    kBalanceAndHighPlace,
    kBalanceAndMidPlace,
    kSmallDrivebaseMovement,
    kSmallArmMovement
  };

  

  
};

