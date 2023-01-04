#include "Robot.h"
#include "behaviours/DriveTeleopBehaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <behaviour/BehaviourScheduler.h>

using namespace frc;
using namespace wom;
using namespace behaviour;

double currentTimeStamp;
double lastTimeStamp;
double dt;

void Robot::RobotInit() {
  driver = new frc::XboxController(0);

  ShooterParams shooterParams{map.shooter.shooterGearbox, map.shooter.pid, map.shooter.currentLimit};
  shooter = new Shooter("shooter", shooterParams);
  
  BehaviourScheduler::GetInstance()->Register(shooter);
  shooter->SetDefaultBehaviour([this]() {
    return make<ShooterConstant>(shooter, 0_V);
  });

  drivetrain = new Drivetrain("drivetrain", map.drivetrain.config);
  drivetrain->GetConfig().gyro->Reset();
  drivetrain->GetConfig().rightDrive.transmission->SetInverted(true);
  drivetrain->GetConfig().leftDrive.encoder->ZeroEncoder();
  drivetrain->GetConfig().rightDrive.encoder->ZeroEncoder();
  BehaviourScheduler::GetInstance()->Register(drivetrain);
  // drivetrain->SetDefaultBehaviour([this]() {
  //   return make<DriveTeleopBehaviour>(drivetrain, driver);
  // });

  waspDrive = new WaspDrive("wasp drive", map.waspDriveSystem.config);

}
void Robot::RobotPeriodic() {
  shooter->OnUpdate(20_ms);
  drivetrain->OnUpdate(20_ms);
  
  BehaviourScheduler::GetInstance()->Tick();

  nt::NetworkTableInstance::GetDefault().GetEntry("/encoder").SetDouble(drivetrain->GetConfig().leftDrive.encoder->GetEncoderRawTicks());
}

void Robot::AutonomousInit() {
  // BehaviourScheduler::GetInstance()->Schedule(
  //   make<DrivetrainDriveDistance>(drivetrain, 3.14_m / 2) 
  //   // make<DrivetrainTurnToAngle>(drivetrain, -45_deg) <<
  //   // make<DrivetrainDriveDistance>(drivetrain, 1_m) <<
  //   // make<DrivetrainTurnToAngle>(drivetrain, 180_deg) <<
  //   // make<DrivetrainDriveDistance>(drivetrain, 1_m)
  //   // make<ShooterSpinup>(shooter, 1500_rpm)
  //   // << make<WaitTime>(1_s)
  //   // << make<ShooterSpinup>(shooter, 2000_rpm)
  //   // << make<WaitTime>(1_s)
  //   // << make<ShooterSpinup>(shooter, 3000_rpm)
  // );

}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // BehaviourScheduler::GetInstance()->Schedule(
  //   make<DriveTeleopBehaviour>(drivetrain, driver)
  // );
}

void Robot::TeleopPeriodic() {
  // double leftSpeed = wom::deadzone(driver->GetLeftY(), 0.05);
  // double rightSpeed = wom::deadzone(driver->GetRightY(), 0.05);
  // double dropSpeed = wom::deadzone(driver->GetLeftX(), 0.05);

  // units::meters_per_second_t leftPower = leftSpeed * 2_mps;
  // units::meters_per_second_t dropPower = dropSpeed * 2_mps;

  // waspDrive->SetVelocity(frc::ChassisSpeeds {
  //   leftSpeed,
  //   dropSpeed,
  //   0_deg
  // })
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}