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

// #define USING_XBOX

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
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->InterruptAll(); // removes all previously scheduled behaviours

  swerve->OnStart();

  swerve->ZeroWheels();


  #ifdef USING_XBOX
    // Swervedrivebase grid poses
    // UP D-BAD
    map.controllers.driver.POV(0, &loop).Rising().IfHigh([sched, this]() {
    if (map.controllers.driver.GetRightBumper()) {
      if (map.controllers.driver.GetLeftBumper()){
        sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.centreGrid2, false)); // central grid
      } else {
        sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.outerGrid3, false)); // Outer Grid 3 (furthest from centre)
      }
    } else {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.innerGrid1, false)); // Inner Grid 1 (furthest from centre)
    }
  });
  // RIGHT D-PAD
  map.controllers.driver.POV(90, &loop).Rising().IfHigh([sched, this]() {
    if (map.controllers.driver.GetRightBumper()) {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.outerGrid2, false)); // Outer Grid 2
    } else {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.innerGrid2, false)); // Inner Grid 2
    }
  });
  // DOWN D-PAD
  map.controllers.driver.POV(180, &loop).Rising().IfHigh([sched, this]() {
    if (map.controllers.driver.GetRightBumper()) {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.outerGrid1, false)); // Outer Grid 1 (closest to centre)
    } else{
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.innerGrid3, false)); // Inner Grid 3 (closest to centre)
    }
  });
  // LEFT D-PAD
  map.controllers.driver.POV(270, &loop).Rising().IfHigh([sched, this]() {
    if (map.controllers.driver.GetRightBumper()) {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.centreGrid3, false)); // Community Grid 3 (outer grid side)
    } else {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.centreGrid1, false)); // Community Grid 1 (inner grid side)
    }
  });

  map.controllers.driver.X(&loop).Rising().IfHigh([sched, this]() {
    sched->Schedule(make<XDrivebase>(swerve));
    map.swerveTable.swerveDriveTable->GetEntry("IsX-ed").SetBoolean(true);
  });
  map.controllers.driver.B(&loop).Rising().IfHigh([sched, this]() {
    swerve->GetActiveBehaviour()->Interrupt();
    map.swerveTable.swerveDriveTable->GetEntry("IsX-ed").SetBoolean(false);
  });
  map.controllers.driver.A(&loop).Rising().IfHigh([sched, this]() {
    sched->Schedule(make<DrivebaseBalance>(swerve, &map.swerveBase.gyro));
  });
  map.controller.driver.Y(&loop).Rising().IfHigh([sched, this]() {
    swerve.ToggleFieldRelative;
  })
  #else
    /*
    translation
    rotation
    
    auto balance
    lock wheels
    cancel behaviour
    
    align forward to nearest
    align backwards to nearest
    
    field relative
    robot orientated

    intake up
    intake down
    arm override
    elevator overide
    arm lock
    elevator lock 

    */
    map.controllers.driver.Triangle(&loop).Rising().IfHigh([sched, this] () {
      // lock wheels
      sched->Schedule(make<XDrivebase>(swerve));
      map.swerveTable.swerveDriveTable->GetEntry("IsX-ed").SetBoolean(true);
    })
    map.controllers.driver.Square(&loop).Rising().IfHigh([sched, this] () {
      // align forward to nearest
    })
    map.controllers.driver.Cross(&loop).Rising().IfHigh([sched, this] () {
      // align backward to neaest
    })
    map.controllers.driver.Circle(&loop).Rising().IfHigh([sched, this] () {
      // cancel current behaviour
      swerve->GetActiveBehaviour()->Interrupt();
      map.swerveTable.swerveDriveTable->GetEntry("IsX-ed").SetBoolean(false);
    })
  
    map.controllers.driver.PS(&loop).Rising().IfHigh([sched, this]() {
      // auto balance behaviour
      sched->Schedule(make<DrivebaseBalance>(swerve, &map.swerveBase.gyro));
    });
  
    map.controllers.driver.Share(&loop).Rising().IfHigh([sched, this]() {
      // field relative
      swerve->SetFieldRelative();
    });
    map.controllers.driver.Options(&loop).Rising().IfHigh([sched, this]() {
      // robot relative
      swerve->SetRobotRelative();
    });
    
    map.controllers.driver.POV(0, &loop).Rising().IfHigh([sched, this]() {
      // intake up
    });
    map.controllers.driver.POV(180, &loop).Rising().IfHigh([sched, this]() {
      // intake down
    });
  
    // arm override
    // elevator override
    // arm lock
    // elevator lock
  #endif

  swerve->OnStart();

}

void Robot::TeleopPeriodic() { }

void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() { }
void Robot::TestPeriodic() { }
