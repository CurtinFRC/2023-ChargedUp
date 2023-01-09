#include "Robot.h"

#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace behaviour;

units::second_t lastPeriodic;

void Robot::RobotInit() {
  lastPeriodic = wom::now();
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
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

#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <networktables/NetworkTableInstance.h>
#include "Armavator.h"
#include "ControlUtil.h"

auto simTable = nt::NetworkTableInstance::GetDefault().GetTable("/sim");

struct SimConfig {
};
SimConfig *simConfig;

units::second_t lastSimPeriodic{0};

void Robot::SimulationInit() {
  simConfig = new SimConfig{

  };

  lastSimPeriodic = wom::now();
}

void Robot::SimulationPeriodic() {
  auto dt = wom::now() - lastSimPeriodic;

  auto batteryVoltage = frc::sim::BatterySim::Calculate({
  });
  frc::sim::RoboRioSim::SetVInVoltage(batteryVoltage);
  simTable->GetEntry("batteryVoltage").SetDouble(batteryVoltage.value());

  lastSimPeriodic = wom::now();
}