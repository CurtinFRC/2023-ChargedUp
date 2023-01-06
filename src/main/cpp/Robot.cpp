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

  mecanumDrivebase = new MecanumDrivebase(map.mecanumDriveSystem.config);
  BehaviourScheduler::GetInstance()->Register(mecanumDrivebase);
  mecanumDrivebase->SetDefaultBehaviour([this]() {
    return make<ManualDrivebase>(mecanumDrivebase, &map.controllers.driver);
  });

  arm = new Arm(map.arm.config);
  BehaviourScheduler::GetInstance()->Register(arm);
  map.arm.config.gearbox.transmission->SetInverted(true);

  climber = new Climber(map.climber.config);
  BehaviourScheduler::GetInstance()->Register(climber);
}
void Robot::RobotPeriodic() {
  /* Update the intake */
  intake->OnUpdate(20_ms);
  mecanumDrivebase->OnUpdate(20_ms);
  arm->OnUpdate(20_ms);
  climber->OnUpdate(20_ms);

  BehaviourScheduler::GetInstance()->Tick();
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  auto sched = BehaviourScheduler::GetInstance();
  sched->Schedule(make<IntakeBehaviour>(intake, &map.controllers.coDriver));
  sched->Schedule(make<ClimberBehaviour>(climber, &map.controllers.coDriver));
  sched->Schedule(make<ManualDrivebase>(mecanumDrivebase, &map.controllers.driver));
}
void Robot::TeleopPeriodic() { 
  // if(map.controllers.driver.GetAButton())
  //   intake->SetIntaking();
  // if(map.controllers.driver.GetBButton())
  //   intake->SetOuttaking();

    double l_x = map.controllers.driver.GetLeftX();
    double l_y = map.controllers.driver.GetLeftY();
    double r_x = map.controllers.driver.GetRightX();

    // deals with controller deadzones
    if (-driverDeadzone <= l_x && l_x <= driverDeadzone) {l_x = 0;}
    if (-driverDeadzone <= l_y && l_y <= driverDeadzone) {l_y = 0;}
    if (-turningDeadzone <= r_x && r_x <= turningDeadzone) {r_x = 0;}

    mecanumDrivebase->SetVelocity(frc::ChassisSpeeds {
        l_x * maxMovementMagnitude,
        l_y * maxMovementMagnitude,
        r_x * 180_deg / 1_s
    });



  if (map.controllers.driver.GetAButton())
    arm->SetAngle(0_deg);
  // if (map.controllers.driver.GetBButton())
  //   arm->SetAngle(30_deg);
  // if (map.controllers.driver.GetXButton())
  //   arm->SetAngle(45_deg);
  if (map.controllers.driver.GetYButton())
    arm->SetAngle(90_deg);
}


void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

/* SIMULATION */

// #include <frc/simulation/DIOSim.h>
// #include <networktables/NetworkTableInstance.h>

// frc::sim::DIOSim *_sim_limit;

// void Robot::SimulationInit() {
//   _sim_limit = new frc::sim::DIOSim(map.arm.limitSwitch);
// }

// units::radian_t _sim_arm_angle{0};

// void Robot::SimulationPeriodic() {
//   _sim_arm_angle += map.arm.gearbox.motor.Speed(90_N * units::math::cos(_sim_arm_angle) * 1_m, map.arm.controller.GetVoltage()) * 20_ms;
  
//   if (_sim_arm_angle <= 0_rad) {
//     _sim_arm_angle = 0_rad;
//     _sim_limit->SetValue(true);
//   } else {
//     _sim_limit->SetValue(false);
//   }

//   if (_sim_arm_angle >= 90_deg) {
//     _sim_arm_angle = 90_deg;
//   }

//   map.arm.encoder.SetTurns(_sim_arm_angle);

//   nt::NetworkTableInstance::GetDefault().GetEntry("arm/angle").SetDouble(_sim_arm_angle.convert<units::degree>().value());
// }