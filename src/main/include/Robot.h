#pragma once

#include <string>
#include "RobotMap.h"
#include "Intake.h"

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "behaviours/ClimberBehaviour.h"
#include "behaviours/DrivebaseBehaviour.h"
#include "behaviours/IntakeBehaviours.h"

#include "Climber.h"
#include "Intake.h"
#include "Arm.h"

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
  RobotMap map;
  Intake *intake;
  MecanumDrivebase *mecanumDrivebase;
  Arm *arm;
  Climber *climber;

  const double driverDeadzone = 0.05;
  const double turningDeadzone = 0.1;
  const units::meters_per_second_t maxMovementMagnitude = 1_mps;

};
