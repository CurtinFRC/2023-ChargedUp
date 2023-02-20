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

  map.swerveBase.moduleConfigs[0].turnMotor.encoder->SetEncoderOffset(1.5232429_rad);
  map.swerveBase.moduleConfigs[1].turnMotor.encoder->SetEncoderOffset(2.2488158_rad);
  map.swerveBase.moduleConfigs[2].turnMotor.encoder->SetEncoderOffset(4.445476_rad);
  map.swerveBase.moduleConfigs[3].turnMotor.encoder->SetEncoderOffset(2.0800779_rad);


  map.swerveBase.gyro.Reset();

  swerve = new wom::SwerveDrive(map.swerveBase.config, frc::Pose2d());
  // map.swerveBase.moduleConfigs[1].turnMotor.transmission->SetInverted(true);
  // map.swerveBase.moduleConfigs[3].turnMotor.transmission->SetInverted(true);
  BehaviourScheduler::GetInstance()->Register(swerve);
  swerve->SetDefaultBehaviour([this]() {
    return make<ManualDrivebase>(swerve, &map.controllers.driver);
  });

  vision = new Vision(&map.vision.config);
  BehaviourScheduler::GetInstance()->Register(vision);
  vision->SetDefaultBehaviour([this]() {
    return make<VisionBehaviour>(vision, swerve, &map.controllers.codriver);
  });


  //creates an instance of the armavator that can be used
  // armavator = new Armavator(map.armavator.arm.gearbox, map.armavator.elevator.gearbox, map.armavator.config);
  // BehaviourScheduler::GetInstance()->Register(armavator);
  // armavator->SetDefaultBehaviour([this]() {
  //   //sets default behaviour class
  //   return make<ArmavatorManualBehaviour>(armavator, map.controllers.codriver);
  // });

  // gripper = new Gripper(map.gripper.config);
  // BehaviourScheduler::GetInstance()->Register(gripper);
  // gripper->SetDefaultBehaviour([this]() {
  //   return make<GripperBehaviour>(gripper, map.controllers.codriver);
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
  swerve->OnStart();
  swerve->ResetPose(frc::Pose2d());
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->Schedule(Drive(swerve, &map.swerveBase.gyro));
 }

void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() {
  loop.Clear();
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->InterruptAll(); // removes all previously scheduled behaviours

  swerve->OnStart();
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

  map.armTable.armManualTable->GetEntry("armSetpoint").SetDouble(_armSetpoint.convert<units::degree>().value());
  map.armTable.armManualTable->GetEntry("elevatorSetpoint").SetDouble(_elevatorSetpoint.convert<units::meter>().value());
}

void Robot::DisabledInit() { 
  // map.controlSystem.pcmCompressor.Disable();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() { }
void Robot::TestPeriodic() { }
