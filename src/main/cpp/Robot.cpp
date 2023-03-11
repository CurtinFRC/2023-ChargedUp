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

#include <cameraserver/CameraServer.h>

#include "Auto.h"

using namespace frc;
using namespace behaviour;

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  cs::UsbCamera camera = CameraServer::StartAutomaticCapture();
  camera.SetResolution(640, 480);
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("Rectangle", 640, 480);

  lastPeriodic = wom::now();

  map.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(4.11207_rad);
  map.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(1.32638_rad);
  map.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(1.67817_rad);
  map.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(0.60899_rad);

  // map.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0_rad);
  // map.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(0_rad);
  // map.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(0_rad);
  // map.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(0_rad);

  map.swerveBase.gyro.Reset();

  swerve = new wom::SwerveDrive(map.swerveBase.config, frc::Pose2d());
  BehaviourScheduler::GetInstance()->Register(swerve);
  swerve->SetDefaultBehaviour([this]() {
    return make<ManualDrivebase>(swerve, &map.controllers.driver);
  });

  vision = new Vision(&map.vision.config);
  BehaviourScheduler::GetInstance()->Register(vision);
  vision->SetDefaultBehaviour([this]() {
    return make<VisionBehaviour>(vision, swerve, &map.controllers.codriver);
  });

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

  swerve->OnUpdate(dt);

  map.swerveTable.swerveDriveTable->GetEntry("frontLeftEncoder").SetDouble(map.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  map.swerveTable.swerveDriveTable->GetEntry("frontRightEncoder").SetDouble(map.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  map.swerveTable.swerveDriveTable->GetEntry("backLeftEncoder").SetDouble(map.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  map.swerveTable.swerveDriveTable->GetEntry("backRightEncoder").SetDouble(map.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());

  std::optional<units::meter_t> distance = map.gripper.gamepiecePresence.GetDistance();
  if (distance.has_value())
    nt::NetworkTableInstance::GetDefault().GetTable("TOF")->GetEntry("distance").SetDouble(distance.value().value());
  else
    nt::NetworkTableInstance::GetDefault().GetTable("TOF")->GetEntry("distance").SetDouble(-1);

  armavator->OnUpdate(dt);
  sideIntake->OnUpdate(dt);
  gripper->OnUpdate(dt);
}

void Robot::AutonomousInit() {
  swerve->OnStart();
  swerve->ResetPose(frc::Pose2d());
  // swerve->ResetPose(frc::Pose2d());  
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  // sched->Schedule(ForwardDrive(Drivebase{swerve, &map.swerveBase.gyro}, armavator));
  sched->Schedule(Single(Drivebase{swerve,  &map.swerveBase.gyro}, armavator, gripper, true, StartingConfig::Bottom, EndingConfig::Dock));
  // sched->Schedule(Balence(Drivebase{swerve, &map.swerveBase.gyro}, armavator));
}

void Robot::AutonomousPeriodic() {
  map.swerveTable.swerveDriveTable->GetEntry("x_pos").SetDouble(swerve->GetPose().X().convert<units::inch>().value());
  map.swerveTable.swerveDriveTable->GetEntry("y_pos").SetDouble(swerve->GetPose().Y().convert<units::inch>().value());
}

void Robot::TeleopInit() {
  loop.Clear();
  sched = BehaviourScheduler::GetInstance();
  sched->InterruptAll(); // removes all previously scheduled behaviours

  swerve->OnStart();
  armavator->OnStart();
}

void Robot::TeleopPeriodic() {
  // if (map.controllers.driver.GetYButton()){ // Sets to Field Relative control mode
  //   swerve->SetIsFieldRelative(true);
  // }
  // if (map.controllers.driver.GetYButton()){ // Sets to Robot Relative control mode
  //   swerve->SetIsFieldRelative(false);
  // }
  // if (map.controllers.driver.GetAButtonPressed()){ // Initiates behaviour for aligning to nearest grid position
  //   std::vector<frc::Pose2d*> blueGridPoses = {
  //     &map.bluePoses.innerGrid1, &map.bluePoses.innerGrid2, &map.bluePoses.innerGrid3,
  //     &map.bluePoses.centreGrid1, &map.bluePoses.centreGrid2, &map.bluePoses.centreGrid3,
  //     &map.bluePoses.outerGrid1, &map.bluePoses.outerGrid2, &map.bluePoses.outerGrid3
  //   };
  //   sched->Schedule(make<AlignDrivebaseToNearestGrid>(swerve, blueGridPoses));
  // }
  // if (map.controllers.driver.GetAButtonPressed()){ // Lock the wheels
  //   sched->Schedule(make<XDrivebase>(swerve));
  //   // map.swerveTable.swerveDriveTable->GetEntry("IsX-ed").SetBoolean(true);
  // }
  // if (map.controllers.driver.GetAButtonPressed()){ // Stop all current behaviours, and return to default (manualDrivebase)
  //   swerve->GetActiveBehaviour()->Interrupt();
  //   // map.swerveTable.swerveDriveTable->GetEntry("IsX-ed").SetBoolean(false);
  // }
  // if (map.controllers.driver.GetAButtonPressed()){ // Initiates behaviour for balancing on the chargestation
  //   sched->Schedule(make<DrivebaseBalance>(swerve, &map.swerveBase.gyro));
  // }
  // if (map.controllers.driver.GetLeftTriggerAxis() > 0.5){
  //   swerve->ResetPose({72.061_in, 20.208_in, 0_deg});
  // }

  map.swerveTable.swerveDriveTable->GetEntry("x").SetDouble(swerve->GetPose().X().convert<units::inch>().value());
  map.swerveTable.swerveDriveTable->GetEntry("x").SetDouble(swerve->GetPose().Y().convert<units::inch>().value());



  // std::cout << "Elevator reading: " << map.armavator.arm.leftEncoder

  // if (map.controllers.codriver.GetXButtonPressed()) {
  //   if (compressorToggle) {
  //     compressorToggle = false;
  //   } else {
  //     compressorToggle = true;
  //   }
  // } 

  // if (map.controllers.driver.GetBButtonPressed()) {
  //   if (compressorToggle) {
  //     compressorToggle = false;
  //   } else {
  //     compressorToggle = true;
  //   }
  // } 

  // if (compressorToggle) {
  //   map.controlSystem.pcmCompressor.EnableDigital();
  //     // std::cout << "compressor true" << std::endl;
  // } else {
  //   map.controlSystem.pcmCompressor.Disable();
  //     // std::cout << "compressor false" << std::endl;
  // }
}


void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() { }
void Robot::TestPeriodic() { }


