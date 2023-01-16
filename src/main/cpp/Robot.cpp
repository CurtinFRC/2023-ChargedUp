#include "Robot.h"

using namespace frc;
using namespace behaviour;

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  lastPeriodic = wom::now();

  armavator = new Armavator(map.armavator.arm.motor1, map.armavator.elevator.motor2);
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
  
  loop.Poll();
  BehaviourScheduler::GetInstance()->Tick();

  armavator->OnUpdate(dt);
  swerve->OnUpdate(dt);
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() {
  loop.Clear();

  //Creates an instance of a behavior scheduler
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
}

void Robot::TeleopPeriodic() {
  // if(!map.controllers.codriver.GetAButton() && !map.controllers.codriver.GetBButton() && !map.controllers.codriver.GetXButton() && !map.controllers.codriver.GetYButton()) {
  //   map.armavator.arm.motor1.SetVoltage(0_V);
  //   map.armavator.elevator.motor2.SetVoltage(0_V);
  // } else{
  //   if(map.controllers.codriver.GetAButton()) {
  //     map.armavator.arm.motor1.SetVoltage(5_V);
  //   } else if (map.controllers.codriver.GetBButton()) {
  //     map.armavator.arm.motor1.SetVoltage(-5_V);
  //   }
  //   if(map.controllers.codriver.GetXButton()) {
  //     map.armavator.elevator.motor2.SetVoltage(5_V);
  //   } else if (map.controllers.codriver.GetYButton()) {
  //     map.armavator.elevator.motor2.SetVoltage(-5_V);
  //   }
  // }
 }

void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() { }
void Robot::TestPeriodic() { }