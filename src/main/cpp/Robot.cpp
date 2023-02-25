#include "Robot.h"
#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"

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

  map.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(1.5232429_rad);
  map.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(2.2488158_rad);
  map.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(4.445476_rad);
  map.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(2.0800779_rad);


  map.swerveBase.gyro.Reset();


  // map.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(2.4743_rad);
  // map.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(2.9774567_rad);
  // map.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(2.0862_rad);
  // map.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(4.9486_rad);

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
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();
  
  loop.Poll();
  BehaviourScheduler::GetInstance()->Tick();

  map.swerveTable.swerveDriveTable->GetEntry("frontLeftEncoder").SetDouble(map.swerveBase.moduleConfigs[0].turnMotor.encoder->GetEncoderPosition().value());
  map.swerveTable.swerveDriveTable->GetEntry("frontRightEncoder").SetDouble(map.swerveBase.moduleConfigs[1].turnMotor.encoder->GetEncoderPosition().value());
  map.swerveTable.swerveDriveTable->GetEntry("backRightEncoder").SetDouble(map.swerveBase.moduleConfigs[2].turnMotor.encoder->GetEncoderPosition().value());
  map.swerveTable.swerveDriveTable->GetEntry("backLeftEncoder").SetDouble(map.swerveBase.moduleConfigs[3].turnMotor.encoder->GetEncoderPosition().value());

  swerve->OnUpdate(dt);

  // map.armTable.armManualTable->GetEntry("arm").SetDouble(map.armavator.arm.motor.GetSupplyCurrent());
  // map.armTable.armManualTable->GetEntry("elv").SetDouble(map.armavator.elevator.motor.GetSupplyCurrent());
  // armavator->OnUpdate(dt);
  // map.intakeTable.intakeTable->GetEntry("state").SetString(sideIntake->GetState());
  // sideIntake->OnUpdate(dt);

  // gripper->OnUpdate(dt);
  // auto visionPose = vision->OnUpdate(dt);

  // if (visionPose.has_value()){
  //   swerve->AddVisionMeasurement(visionPose.value().first.ToPose2d(), visionPose.value().second);
  // }

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
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  swerve->OnStart();
  swerve->ResetPose(frc::Pose2d());
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->Schedule(Single(Drivebase{swerve, &map.swerveBase.gyro}, true, StartingConfig::Top, EndingConfig::Dock));
 }

void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();

  loop.Clear();
  sched = BehaviourScheduler::GetInstance();
  sched->InterruptAll(); // removes all previously scheduled behaviours

  swerve->OnStart();
  armavator->OnStart();
}

void Robot::TeleopPeriodic() {
  if (map.controllers.driver.GetMiniLeftButtonPressed()){ // Sets to Field Relative control mode
    swerve->SetIsFieldRelative(true);
  }
  if (map.controllers.driver.GetMiniRightButtonPressed()){ // Sets to Robot Relative control mode
    swerve->SetIsFieldRelative(false);
  }
  if (map.controllers.driver.GetCPAD_BottomPressed()){ // Initiates behaviour for aligning to nearest grid position
    std::vector<frc::Pose2d*> blueGridPoses = {
      &map.bluePoses.innerGrid1, &map.bluePoses.innerGrid2, &map.bluePoses.innerGrid3,
      &map.bluePoses.centreGrid1, &map.bluePoses.centreGrid2, &map.bluePoses.centreGrid3,
      &map.bluePoses.outerGrid1, &map.bluePoses.outerGrid2, &map.bluePoses.outerGrid3
    };
    sched->Schedule(make<AlignDrivebaseToNearestGrid>(swerve, blueGridPoses));
  }
  if (map.controllers.driver.GetCPAD_TopPressed()){ // Lock the wheels
    sched->Schedule(make<XDrivebase>(swerve));
    map.swerveTable.swerveDriveTable->GetEntry("IsX-ed").SetBoolean(true);
  }
  if (map.controllers.driver.GetCPAD_RightPressed()){ // Stop all current behaviours, and return to default (manualDrivebase)
    swerve->GetActiveBehaviour()->Interrupt();
    map.swerveTable.swerveDriveTable->GetEntry("IsX-ed").SetBoolean(false);
  }
  if (map.controllers.driver.GetLogoButtonPressed()){ // Initiates behaviour for balancing on the chargestation
    sched->Schedule(make<DrivebaseBalance>(swerve, &map.swerveBase.gyro));
  }
  if (map.controllers.driver.GetLeftBumperPressed()){ // Initiates behaviour for balancing on the chargestation
    swerve->ResetPose(frc::Pose2d{0_m, 0_m, 0_deg});
  }
  // std::cout << "Elevator reading: " << map.armavator.arm.leftEncoder

  if (map.controllers.codriver.GetXButtonPressed()) {
    if (compressorToggle) {
      compressorToggle = false;
    } else {
      compressorToggle = true;
    }
  } 

  if (compressorToggle) {
    map.controlSystem.pcmCompressor.EnableDigital();
      std::cout << "compressor true" << std::endl;
  } else {
    map.controlSystem.pcmCompressor.Disable();
      std::cout << "compressor false" << std::endl;
  }

  map.armTable.armManualTable->GetEntry("armSetpoint").SetDouble(_armSetpoint.convert<units::degree>().value());
  map.armTable.armManualTable->GetEntry("elevatorSetpoint").SetDouble(_elevatorSetpoint.convert<units::meter>().value());
}


void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() { }
void Robot::TestPeriodic() { }


