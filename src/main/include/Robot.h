#pragma once

#include "RobotMap.h"
#include "Vision.h"

#include <string>

#include <ctre/Phoenix.h>
#include <frc/TimedRobot.h>
#include <frc/event/EventLoop.h>
#include "ControlUtil.h"


#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/event/BooleanEvent.h>

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

  // void SimulationInit() override;
  // void SimulationPeriodic() override;

 private:
  frc::EventLoop loop;
  
  Armavator *armavator;
  RobotMap map;
  wom::SwerveDrive *swerve;
  Vision *vision;
  behaviour::BehaviourScheduler *sched;
  
  //SwerveModuleTest *swerveModule;
  // SideIntake *sideIntake;
  // Gripper *gripper;

  bool compressorToggle = false;

  units::meter_t _elevatorSetpoint = 0_m;
  units::radian_t _armSetpoint = 0_deg;

};