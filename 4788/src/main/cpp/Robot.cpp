#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace wom;

double currentTimeStamp;
double lastTimeStamp;
double dt;

void Robot::RobotInit() {
  ShooterParams shooterParams{map.shooter.shooterGearbox, map.shooter.pid, map.shooter.currentLimit};
  shooter = new Shooter(shooterParams);
}
void Robot::RobotPeriodic() {
  shooter->OnUpdate(20_ms);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  shooter->SetPID(4500_rpm);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}