#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <behaviour/BehaviourScheduler.h>

using namespace frc;
using namespace wom;
using namespace behaviour;

double currentTimeStamp;
double lastTimeStamp;
double dt;

void Robot::RobotInit() { }
void Robot::RobotPeriodic() {
  BehaviourScheduler::GetInstance()->Tick();
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() { }

void Robot::TeleopPeriodic() { }

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}