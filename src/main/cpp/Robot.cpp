#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"

using namespace frc;
using namespace behaviour;

void Robot::RobotInit() {
  /* Create a new intake */
  intake = new Intake(map.intake.config);
  BehaviourScheduler::GetInstance()->Register(intake);

  arm = new Arm(map.arm.config);
  BehaviourScheduler::GetInstance()->Register(arm);
  
  mecanumDrivebase = new MecanumDrivebase(map.mecanumDriveSystem.config);
  BehaviourScheduler::GetInstance()->Register(mecanumDrivebase);
}
void Robot::RobotPeriodic() {
  /* Update the intake */
  intake->OnUpdate(20_ms);
  arm->OnUpdate(20_ms);
  mecanumDrivebase->OnUpdate(20_ms);
  BehaviourScheduler::GetInstance()->Tick();
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() { }

void Robot::TeleopPeriodic() { 
  // if(map.controllers.driver.GetAButton())
  //   intake->SetIntaking();
  // if(map.controllers.driver.GetBButton())
  //   intake->SetOuttaking();

  if (map.controllers.driver.GetAButton())
    arm->SetAngle(0_deg);
  if (map.controllers.driver.GetBButton())
    arm->SetAngle(30_deg);
  if (map.controllers.driver.GetXButton())
    arm->SetAngle(45_deg);
  if (map.controllers.driver.GetYButton())
    arm->SetAngle(90_deg);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

/* SIMULATION */

#include <frc/simulation/DIOSim.h>

// frc::sim::DIOSim *simArmLimitSwitch;
struct SimConfig {
  struct ArmSimConfig {
    frc::sim::DIOSim limitSwitch;
    ::sim::ArmSim armSim;
  };
  ArmSimConfig armSim;
};
SimConfig *simConfig;

void Robot::SimulationInit() {
  // simArmLimitSwitch = new frc::sim::DIOSim(map.arm.limitSwitch);
  simConfig = new SimConfig {
    SimConfig::ArmSimConfig {
      frc::sim::DIOSim(map.arm.limitSwitch),
      ::sim::ArmSim{
        map.arm.gearbox.motor,
        10_kg, 1_m
      }
    }
  };
}

void Robot::SimulationPeriodic() {
  simConfig->armSim.armSim.Update(map.arm.controller.GetVoltage(), 20_ms);
  simConfig->armSim.limitSwitch.SetValue(simConfig->armSim.armSim.IsLimit());
  map.arm.encoder.SetTurns(simConfig->armSim.armSim.angle);
}