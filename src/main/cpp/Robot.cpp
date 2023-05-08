#include "Robot.h"
#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"
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
  //create a camera and public the stream to shuffleboard camera server
  cs::UsbCamera camera = CameraServer::StartAutomaticCapture();
  camera.SetResolution(640, 480);
  cs::CvSink cvSink = frc::CameraServer::GetVideo();
  cs::CvSource outputStream = frc::CameraServer::PutVideo("Rectangle", 640, 480);

  lastPeriodic = wom::now();

  //sets the offsets of the swerve cancoders to retune these values set the offsets to 0 and record the values
  map.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(4.11207_rad);
  map.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(1.32638_rad);
  map.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(1.67817_rad);
  map.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(0.60899_rad);

  // map.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(0_rad);
  // map.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(0_rad);
  // map.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(0_rad);
  // map.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(0_rad);

  //the start of selecting which auto mode, was never successfully implemented 
  m_chooser.SetDefaultOption(kLowPlace, kLowPlace);
  m_chooser.AddOption(kLowPlaceTaxi, kLowPlaceTaxi);
  m_chooser.AddOption(kHighPlaceTaxi, kHighPlaceTaxi);
  m_chooser.AddOption(kHighPlace, kHighPlace);
  m_chooser.AddOption(kPlaceDock, kPlaceDock);
  m_chooser.AddOption(kDock, kDock);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //reset the gyro when the robot restarts 
  map.swerveBase.gyro.Reset();
  

  swerve = new wom::SwerveDrive(map.swerveBase.config, frc::Pose2d());
  BehaviourScheduler::GetInstance()->Register(swerve);
  swerve->SetDefaultBehaviour([this]() {
    return make<ManualDrivebase>(swerve, &map.controllers.driver);
  });

  vision = new Vision(&map.vision.config);
  // BehaviourScheduler::GetInstance()->Register(vision);
  // vision->SetDefaultBehaviour([this]() {
  //   return make<VisionBehaviour>(vision, swerve, &map.controllers.codriver);
  // });

  vision->table->GetEntry("goToPoseX").SetDouble(0);
  vision->table->GetEntry("goToPoseY").SetDouble(0);
  vision->table->GetEntry("goToPoseRotation").SetDouble(0);


  armavator = new Armavator(map.armavator.arm.leftGearbox, map.armavator.arm.rightGearbox, map.armavator.elevator.rightGearbox, map.armavator.elevator.leftGearbox, map.armavator.config);
  BehaviourScheduler::GetInstance()->Register(armavator);
  armavator->SetDefaultBehaviour([this]() {
    return make<ArmavatorManualBehaviour>(armavator, map.controllers.codriver);
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

  //publish the encoder values for the swervebase 
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
  gripper->OnUpdate(dt);
}

void Robot::AutonomousInit() {
  swerve->OnStart();
  swerve->ResetPose(frc::Pose2d()); //reset the current swerve pose 
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->Schedule(PlaceBalence(Drivebase{swerve, &map.swerveBase.gyro}, armavator, gripper)); //schedule the auto to be run
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  loop.Clear();
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->InterruptAll(); // removes all previously scheduled behaviours

  swerve->OnStart();
  armavator->OnStart();

  //when the b button is pressed, stop all currently running behaviours 
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
}

void Robot::TeleopPeriodic() {
  //pushes the swerve position to network tables 
  map.swerveTable.swerveDriveTable->GetEntry("x").SetDouble(swerve->GetPose().X().convert<units::meter>().value());
  map.swerveTable.swerveDriveTable->GetEntry("x").SetDouble(swerve->GetPose().Y().convert<units::meter>().value());
  auto dt = wom::now() - lastPeriodic;

  vision->OnUpdate(dt);

  //when the b button is pressed, interupt currently running behaviours
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


