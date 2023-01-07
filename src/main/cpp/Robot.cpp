#include "Robot.h"

#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace behaviour;

void Robot::RobotInit() {

}

void Robot::RobotPeriodic() {
  BehaviourScheduler::GetInstance()->Tick();
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() { }
void Robot::TeleopPeriodic() { }

void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() { }
void Robot::TestPeriodic() { }

/* SIMULATION */

struct SimConfig {
};
SimConfig *simConfig;

void Robot::SimulationInit() {
  // simArmLimitSwitch = new frc::sim::DIOSim(map.arm.limitSwitch);
  simConfig = new SimConfig {
  };
}

void Robot::SimulationPeriodic() {
}