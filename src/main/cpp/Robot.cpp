#include "Robot.h"
#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"
#include "behaviour/VisionBehaviour.h"

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

  swerve->OnUpdate(dt);

  auto visionPose = vision->OnUpdate(dt);
  if (visionPose.has_value()){
    swerve->AddVisionMeasurement(visionPose.value().first.ToPose2d(), visionPose.value().second);
  }
}

void Robot::AutonomousInit() {
  swerve->OnStart();
  swerve->ResetPose(frc::Pose2d());
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->Schedule(Single(Drivebase{swerve, &map.swerveBase.gyro}, true, StartingConfig::Top, EndingConfig::Dock));
 }

void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() {
  loop.Clear();
  sched = BehaviourScheduler::GetInstance();
  sched->InterruptAll(); // removes all previously scheduled behaviours

  swerve->OnStart();
  swerve->ZeroWheels();
  swerve->OnStart();

}

void Robot::TeleopPeriodic() {
  if (map.controllers.driver.GetMiniLeftButtonPressed()){
    swerve->SetIsFieldRelative(true);
  }
  if (map.controllers.driver.GetMiniRightButtonPressed()){
    swerve->SetIsFieldRelative(false);
  }
  if (map.controllers.driver.GetCPAD_BottomPressed()){
    std::vector<frc::Pose2d*> blueGridPoses = {
      &map.bluePoses.innerGrid1, &map.bluePoses.innerGrid2, &map.bluePoses.innerGrid3,
      &map.bluePoses.centreGrid1, &map.bluePoses.centreGrid2, &map.bluePoses.centreGrid3,
      &map.bluePoses.outerGrid1, &map.bluePoses.outerGrid2, &map.bluePoses.outerGrid3
    };
    sched->Schedule(make<AlignDrivebaseToNearestGrid>(swerve, blueGridPoses));
  }
  if (map.controllers.driver.GetCPAD_TopPressed()){
    sched->Schedule(make<XDrivebase>(swerve));
    map.swerveTable.swerveDriveTable->GetEntry("IsX-ed").SetBoolean(false);
  }
  if (map.controllers.driver.GetCPAD_RightPressed()){
    swerve->GetActiveBehaviour()->Interrupt();
    map.swerveTable.swerveDriveTable->GetEntry("IsX-ed").SetBoolean(false);
  }
  if (map.controllers.driver.GetLogoButtonPressed()){
    sched->Schedule(make<DrivebaseBalance>(swerve, &map.swerveBase.gyro));
  }

}

void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() { }
void Robot::TestPeriodic() { }
