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
#include "frc/smartdashboard/SendableChooser.h"
#include "Auto.h"

using namespace frc;
using namespace behaviour;

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";

  std::string m_autoSelected;
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption("h", kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  /*
  https://docs.wpilib.org/en/stable/docs/software/vision-processing/roborio/using-the-cameraserver-on-the-roborio.html
  he following 4 lines of code were absolutely robbed from this website
  */
  cs::UsbCamera camera = CameraServer::StartAutomaticCapture();
  camera.SetResolution(640, 480);
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("Rectangle", 640, 480);

  map.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0.951068_rad);
  map.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(4.41479_rad);
  map.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(4.81669_rad);
  map.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(3.81194_rad);


  m_chooser.SetDefaultOption(kLowPlace, kLowPlace);
  m_chooser.AddOption(kLowPlaceTaxi, kLowPlaceTaxi);
  m_chooser.AddOption(kHighPlaceTaxi, kHighPlaceTaxi);
  m_chooser.AddOption(kHighPlace, kHighPlace);
  m_chooser.AddOption(kPlaceDock, kPlaceDock);
  m_chooser.AddOption(kDock, kDock);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  lastPeriodic = wom::now();
  map.swerveBase.gyro.Reset();

  vision = new Vision(&map.vision.config);
  swerve = new wom::SwerveDrive(map.swerveBase.config, frc::Pose2d());
  BehaviourScheduler::GetInstance()->Register(swerve);
  swerve->SetDefaultBehaviour([this]() {
    return make<ManualDrivebase>(swerve, &map.controllers.driver);
  });

  vision->table->GetEntry("goToPoseX").SetDouble(0);
  vision->table->GetEntry("goToPoseY").SetDouble(0);
  vision->table->GetEntry("goToPoseRotation").SetDouble(0);

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
  vision->OnUpdate(dt);
}

void Robot::AutonomousInit() {
  swerve->OnStart();
  swerve->ResetPose(frc::Pose2d());
  // swerve->ResetPose(frc::Pose2d());  
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  // sched->Schedule(SubsystemTestPlace(armavator));
  // sched->Schedule(Single(Drivebase{swerve,  &map.swerveBase.gyro}, armavator, gripper, true, StartingConfig::Bottom, EndingConfig::Dock));
  // sched->Schedule(ForwardDrive(Drivebase{swerve, &map.swerveBase.gyro}, armavator));
  sched->Schedule(Single(Drivebase{swerve,  &map.swerveBase.gyro}, armavator, gripper, true, StartingConfig::Bottom, EndingConfig::Dock));
  // sched->Schedule(Balence(Drivebase{swerve, &map.swerveBase.gyro}, armavator));

  if (m_autoSelected == "kLowPlace") {

  } else if (m_autoSelected == "lowPlaceTaxi") {

  } else if (m_autoSelected == "highPlaceTaxi") {

  } else if (m_autoSelected == "highPlace") {

  } else if (m_autoSelected == "placeDock") {

  } else if (m_autoSelected == "dock") {

  }
}

void Robot::AutonomousPeriodic() {
  map.swerveTable.swerveDriveTable->GetEntry("x_pos").SetDouble(swerve->GetPose().X().convert<units::inch>().value());
  map.swerveTable.swerveDriveTable->GetEntry("y_pos").SetDouble(swerve->GetPose().Y().convert<units::inch>().value());
}

void Robot::TeleopInit() {
  loop.Clear();
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  // sched = BehaviourScheduler::GetInstance();
  // sched->InterruptAll(); // removes all previously scheduled behaviours
  sched->InterruptAll();

  swerve->OnStart();
  armavator->OnStart();

   map.controllers.driver.B(&loop).Rising().IfHigh([sched, this]() {
    swerve->GetActiveBehaviour()->Interrupt();
  });
  // map.controllers.driver.Y(&loop).Rising().IfHigh([sched, this]() { // temp solutions
  //   swerve->ResetPose(vision->EstimatePose(vision->GetConfig()).ToPose2d());
  // });
  map.controllers.driver.POV(0, &loop).Rising().IfHigh([sched, this]() {
    sched->Schedule(make<AlignDrivebaseToNearestGrid>(swerve, vision, 0));
  });
  map.controllers.driver.POV(270, &loop).Rising().IfHigh([sched, this]() {
    sched->Schedule(make<AlignDrivebaseToNearestGrid>(swerve, vision, -1));
  });
  map.controllers.driver.POV(90, &loop).Rising().IfHigh([sched, this]() {
    sched->Schedule(make<AlignDrivebaseToNearestGrid>(swerve, vision, 1));
  });
  // sched->Schedule(make<ArmavatorGoToAutoSetpoint>(armavator, 0.2_m, 0_deg));
}

void Robot::TeleopPeriodic() {
  map.swerveTable.swerveDriveTable->GetEntry("x").SetDouble(swerve->GetPose().X().convert<units::inch>().value());
  map.swerveTable.swerveDriveTable->GetEntry("x").SetDouble(swerve->GetPose().Y().convert<units::inch>().value());
  auto dt = wom::now() - lastPeriodic;

  vision->OnUpdate(dt);

  // // test function
  // if (map.controllers.driver.GetLeftTriggerAxis() >= 0.5){
  //   vision->table->GetEntry("triggerPressed").SetBoolean(true);
  //   sched->Schedule(make<AlignDrivebaseToNearestGrid>(swerve, vision));
  // }
  // returns to ful manual control
  if (map.controllers.test.GetBButtonPressed()){
    swerve->GetActiveBehaviour()->Interrupt();
  }
}


void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() { }
void Robot::TestPeriodic() {
  auto dt = wom::now() - lastPeriodic;

  vision->OnUpdate(dt);
}


