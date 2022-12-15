#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

double currentTimeStamp;
double lastTimeStamp;
double dt;

void Robot::RobotInit() {
  xboxcontroller = new frc::XboxController(0);
  talonsrx = new TalonSRX(99);
  victorspx = new VictorSPX(99);
  talonfx = new TalonFX(99);
}
void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {
  if (xboxcontroller->GetXButtonPressed())   {
    talonsrx->Set(ControlMode::PercentOutput, 0.6);
  } else if (xboxcontroller->GetYButtonPressed()) {
    victorspx->Set(ControlMode::PercentOutput, 0.6);
  } else if (xboxcontroller->GetAButtonPressed()) {
    talonfx->Set(ControlMode::PercentOutput, 0.6);
  } else {
    talonsrx->Set(ControlMode::PercentOutput, 0);
    victorspx->Set(ControlMode::PercentOutput, 0);
    talonfx->Set(ControlMode::PercentOutput, 0);
  }
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}