#include "Robot.h"

#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace behaviour;

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  lastPeriodic = wom::now();

  arm = new wom::Arm(map.arm.config);
  BehaviourScheduler::GetInstance()->Register(arm);
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  BehaviourScheduler::GetInstance()->Tick();

  arm->OnUpdate(dt);
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() { }
void Robot::TeleopPeriodic() {
  if (map.controllers.driver.GetAButton())
    arm->SetAngle(45_deg);
  if (map.controllers.driver.GetBButton())
    arm->SetAngle(90_deg);
  if (map.controllers.driver.GetXButton())
    arm->SetAngle(135_deg);
  if (map.controllers.driver.GetYButton())
    arm->SetAngle(0_deg);
}

void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() { }
void Robot::TestPeriodic() { }

/* SIMULATION */

#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <networktables/NetworkTableInstance.h>
#include "ControlUtil.h"

static units::second_t lastSimPeriodic{0};
static auto simTable = nt::NetworkTableInstance::GetDefault().GetTable("/sim");

struct SimConfig {
  wom::sim::ArmSim arm;
};
SimConfig *simConfig;

void Robot::SimulationInit() {
  simConfig = new SimConfig{
    wom::sim::ArmSim(map.arm.config)
  };

  lastSimPeriodic = wom::now();
}

void Robot::SimulationPeriodic() {
  auto dt = wom::now() - lastSimPeriodic;

  simConfig->arm.Update(dt);

  auto batteryVoltage = units::math::min(units::math::max(frc::sim::BatterySim::Calculate({
    // simConfig->arm.GetCurrent()
  }), 0_V), 12_V);
  frc::sim::RoboRioSim::SetVInVoltage(batteryVoltage);
  simTable->GetEntry("batteryVoltage").SetDouble(batteryVoltage.value()); 

  lastSimPeriodic = wom::now();
}