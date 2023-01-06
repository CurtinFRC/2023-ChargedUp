#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"

using namespace frc;
using namespace behaviour;

void Robot::RobotInit() { }
void Robot::RobotPeriodic() {
  BehaviourScheduler::GetInstance()->Tick();
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() { }

void Robot::TeleopPeriodic() { 
  if(map.controllers.driver.GetAButton())
    Intake->SetIntaking();
  if(map.controllers.driver.GetBButton())
    Intake->SetOuttaking();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}