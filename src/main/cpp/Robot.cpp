#include "Robot.h"

#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;
using namespace behaviour;

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  lastPeriodic = wom::now();

  armavator = new Armavator(map.armavator.config);
  BehaviourScheduler::GetInstance()->Register(armavator);

  swerve = new wom::SwerveDrive(map.swerveBase.config, frc::Pose2d());
  BehaviourScheduler::GetInstance()->Register(swerve);
  swerve->SetDefaultBehaviour([this]() {
    return make<ManualDrivebase>(swerve, &map.controllers.driver);
  });
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();
  
  BehaviourScheduler::GetInstance()->Tick();

  armavator->OnUpdate(dt);
  swerve->OnUpdate(dt);
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() { }
void Robot::TeleopPeriodic() {
  if (map.controllers.driver.GetAButton())
    armavator->SetPosition({0.5_m, 45_deg});
  if (map.controllers.driver.GetBButton())
    armavator->SetPosition({1_m, 90_deg});
  if (map.controllers.driver.GetXButton())
    armavator->SetPosition({1.0_m, -90_deg});
  if (map.controllers.driver.GetYButton())
    armavator->SetPosition({0_m, 0_deg});
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
  ::sim::ArmavatorSim arm;
  wom::sim::SwerveDriveSim swerveSim; 
};
SimConfig *simConfig;

void Robot::SimulationInit() {
  simConfig = new SimConfig{
    ::sim::ArmavatorSim(map.armavator.config),
    wom::sim::SwerveDriveSim(map.swerveBase.config, 0.5 * 6_lb * 7.5_in * 7.5_in)
  };

  lastSimPeriodic = wom::now();
}

void Robot::SimulationPeriodic() {
  auto dt = wom::now() - lastSimPeriodic;

  simConfig->arm.OnUpdate(dt);
  simConfig->swerveSim.Update(dt);

  auto batteryVoltage = units::math::min(units::math::max(frc::sim::BatterySim::Calculate({
    // simConfig->arm.GetCurrent()
  }), 0_V), 12_V);
  frc::sim::RoboRioSim::SetVInVoltage(batteryVoltage);
  simTable->GetEntry("batteryVoltage").SetDouble(batteryVoltage.value()); 

  lastSimPeriodic = wom::now();
}