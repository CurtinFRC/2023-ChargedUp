#include "Robot.h"
#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"
#include "behaviour/SideIntakeBehaviour.h"
#include "behaviour/GripperBehaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/event/BooleanEvent.h>
#include <units/math.h>
#include <networktables/NetworkTableInstance.h>


#include "Auto.h"

using namespace frc;
using namespace behaviour;

static units::second_t lastPeriodic;

void Robot::RobotInit() {

  lastPeriodic = wom::now();

  map.swerveBase.gyro.Reset();

  // map.swerveBase.moduleConfigs[0].turnMotor.encoder->ZeroEncoder();
  // map.swerveBase.moduleConfigs[1].turnMotor.encoder->ZeroEncoder();
  // map.swerveBase.moduleConfigs[2].turnMotor.encoder->ZeroEncoder();
  // map.swerveBase.moduleConfigs[3].turnMotor.encoder->ZeroEncoder();

  map.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(2.4743_rad);
  map.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(2.9774567_rad);
  map.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(2.0862_rad);
  map.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(4.9486_rad);

  swerve = new wom::SwerveDrive(map.swerveBase.config, frc::Pose2d());
  BehaviourScheduler::GetInstance()->Register(swerve);
  swerve->SetDefaultBehaviour([this]() {
    return make<ManualDrivebase>(swerve, &map.controllers.driver);
  });

  // vision = new Vision(&map.vision.config);
  // BehaviourScheduler::GetInstance()->Register(vision);
  // vision->SetDefaultBehaviour([this]() {
  //   return make<VisionBehaviour>(vision, swerve, &map.controllers.codriver);
  // });

  armavator = new Armavator(map.armavator.arm.leftGearbox, map.armavator.arm.rightGearbox, map.armavator.elevator.rightGearbox, map.armavator.elevator.leftGearbox, map.armavator.config);
  BehaviourScheduler::GetInstance()->Register(armavator);
  armavator->SetDefaultBehaviour([this]() {
    return make<ArmavatorManualBehaviour>(armavator, map.controllers.codriver);
  });

  sideIntake = new SideIntake(map.sideIntake.config);
  BehaviourScheduler::GetInstance()->Register(sideIntake);
  sideIntake->SetDefaultBehaviour([this]() {
    return make<SideIntakeBehaviour>(sideIntake, map.controllers.codriver);
  });

  gripper = new Gripper(map.gripper.config);
  BehaviourScheduler::GetInstance()->Register(gripper);
  gripper->SetDefaultBehaviour([this]() {
    return make<GripperBehaviour>(gripper, map.controllers.codriver);
  });
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();
  
  loop.Poll();
  BehaviourScheduler::GetInstance()->Tick();

  // double FrontRightPosition = std::fmod(map.swerveBase.frontRightCancoder.GetPosition(), 360);
  // double FrontLeftPosition  = std::fmod(map.swerveBase.frontLeftCancoder.GetPosition(), 360);
  // double BackRightPosition = std::fmod(map.swerveBase.backRightCancoder.GetPosition(), 360);
  // double BackLeftPosition = std::fmod(map.swerveBase.backLeftCancoder.GetPosition(), 360);

  // std::cout << "JHJKDGKGK" << map.swerveBase.moduleConfigs[0].turnMotor.encoder->GetAbsoluteEncoderPosition() << std::endl;

  double FrontRightPosition = swerve->GetModuleCANPosition(0);
  double FrontLeftPosition  = swerve->GetModuleCANPosition(1);
  double BackRightPosition = swerve->GetModuleCANPosition(2);
  double BackLeftPosition = swerve->GetModuleCANPosition(3);

  double rawFrontRight = map.swerveBase.frontRightCancoder.GetPosition();
  double rawFrontLeft = map.swerveBase.frontLeftCancoder.GetPosition();
  double rawBackRight = map.swerveBase.backRightCancoder.GetPosition();
  double rawBackLeft = map.swerveBase.backLeftCancoder.GetPosition();

  // map.swerveTable.swerveDriveTable->GetEntry("raw front left").SetDouble(rawFrontLeft);
  // map.swerveTable.swerveDriveTable->GetEntry("raw front right").SetDouble(rawFrontRight);
  // map.swerveTable.swerveDriveTable->GetEntry("raw back left").SetDouble(rawBackLeft);
  // map.swerveTable.swerveDriveTable->GetEntry("raw back right").SetDouble(rawBackRight);

  // map.swerveTable.swerveDriveTable->GetEntry("frontLeftCancoder").SetDouble(FrontLeftPosition* (3.141592 / 180));
  // map.swerveTable.swerveDriveTable->GetEntry("frontRightCancoder").SetDouble(FrontRightPosition* (3.141592 / 180));
  // map.swerveTable.swerveDriveTable->GetEntry("backRightCancoder").SetDouble(BackRightPosition* (3.141592 / 180));
  // map.swerveTable.swerveDriveTable->GetEntry("backLeftCancoder").SetDouble(BackLeftPosition * (3.141592 / 180));

  // map.swerveTable.swerveDriveTable->GetEntry("frontLeftEncoder").SetDouble(map.swerveBase.moduleConfigs[0].turnMotor.encoder->GetAbsoluteEncoderPosition());

  map.swerveTable.swerveDriveTable->GetEntry("frontLeftEncoder").SetDouble(map.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  map.swerveTable.swerveDriveTable->GetEntry("frontRightEncoder").SetDouble(map.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  map.swerveTable.swerveDriveTable->GetEntry("backRightEncoder").SetDouble(map.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  map.swerveTable.swerveDriveTable->GetEntry("backLeftEncoder").SetDouble(map.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());

  swerve->OnUpdate(dt);
  armavator->OnUpdate(dt);
  sideIntake->OnUpdate(dt);
  gripper->OnUpdate(dt);
}

void Robot::AutonomousInit() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  swerve->OnStart(dt);
  swerve->ResetPose(frc::Pose2d());
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->Schedule(Drive(swerve, &map.swerveBase.gyro));
}

void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Clear();
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->InterruptAll(); // removes all previously scheduled behaviours
  swerve->OnStart(dt);
  armavator->OnStart();
}

void Robot::TeleopPeriodic() {
  if (map.controllers.driver.GetXButtonPressed()) {
    if (compressorToggle) {
      compressorToggle = false;
    } else {
      compressorToggle = true;
    }
  } 

  if (compressorToggle) {
    map.controlSystem.pcmCompressor.EnableDigital();
  } else {
    map.controlSystem.pcmCompressor.Disable();
  }



  // map.armTable.armManualTable->GetEntry("armSetpoint").SetDouble(_armSetpoint.convert<units::degree>().value());
  // map.armTable.armManualTable->GetEntry("elevatorSetpoint").SetDouble(_elevatorSetpoint.convert<units::meter>().value());
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() { }
void Robot::TestPeriodic() { }
