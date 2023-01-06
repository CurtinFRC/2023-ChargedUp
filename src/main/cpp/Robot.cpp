#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"

using namespace frc;
using namespace behaviour;

void Robot::RobotInit() {
  /* Create a new intake */
  intake = new Intake(map.intake.config);
  mecanumDrivebase = new MecanumDrivebase(map.mecanumDriveSystem.config);

}
void Robot::RobotPeriodic() {
  /* Update the intake */
  intake->OnUpdate(20_ms);
  mecanumDrivebase->OnUpdate(20_ms);
  BehaviourScheduler::GetInstance()->Tick();
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() { }

void Robot::TeleopPeriodic() {
  /* Control the intake! */
  if (map.controllers.driver.GetAButton())
    intake->SetIntaking();
  if (map.controllers.driver.GetBButton())
    intake->SetOuttaking();
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}