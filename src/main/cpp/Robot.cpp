#include "Robot.h"
#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/event/BooleanEvent.h>
#include <units/math.h>


#include "Auto.h"

using namespace frc;
using namespace behaviour;

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  lastPeriodic = wom::now();

  vision = new Vision(map.vision.config);

  map.swerveBase.gyro.Reset();
  swerve = new wom::SwerveDrive(map.swerveBase.config, frc::Pose2d());
  BehaviourScheduler::GetInstance()->Register(swerve);
  swerve->SetDefaultBehaviour([this]() {
    return make<ManualDrivebase>(swerve, &map.controllers.driver);
  });

  armavator = new Armavator(map.armavator.arm.gearbox, map.armavator.elevator.gearbox, map.armavator.config);
  BehaviourScheduler::GetInstance()->Register(armavator);
  armavator->SetDefaultBehaviour([this]() {
    return make<ArmavatorRawBehaviour>(armavator, map.controllers.codriver);
  });
}

void Robot::RobotPeriodic() {
  auto dt = wom::now() - lastPeriodic;
  lastPeriodic = wom::now();
  
  loop.Poll();
  BehaviourScheduler::GetInstance()->Tick();

  swerve->OnUpdate(dt);

  map.armTable.armManualTable->GetEntry("arm").SetDouble(map.armavator.arm.motor.GetSupplyCurrent());
  map.armTable.armManualTable->GetEntry("elv").SetDouble(map.armavator.elevator.motor.GetSupplyCurrent());
  armavator->OnUpdate(dt);

  auto visionPose = vision->OnUpdate(dt);

  if (visionPose.has_value()){
    swerve->AddVisionMeasurement(visionPose.value().first.ToPose2d(), visionPose.value().second);
  }

}

void Robot::AutonomousInit() {
  swerve->OnStart();
  swerve->ResetPose(frc::Pose2d());
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->Schedule(CircularPathing(swerve));
 }

void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() {
  loop.Clear();
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  sched->InterruptAll();

  swerve->OnStart();

  // Swervedrivebase grid poses
  // UP D-BAD
  map.controllers.driver.POV(0, &loop).Rising().IfHigh([sched, this]() {
    if (map.controllers.driver.GetRightBumper()) {
      if (map.controllers.driver.GetLeftBumper()){
        sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.centreGrid2)); // central grid
      } else {
        sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.outerGrid3)); // Outer Grid 3 (furthest from centre)
      }
    } else {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.innerGrid1)); // Inner Grid 1 (furthest from centre)
    }
  });
  // RIGHT D-PAD
  map.controllers.driver.POV(90, &loop).Rising().IfHigh([sched, this]() {
    if (map.controllers.driver.GetRightBumper()) {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.outerGrid2)); // Outer Grid 2
    } else {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.innerGrid2)); // Inner Grid 2
    }
  });
  // DOWN D-PAD
  map.controllers.driver.POV(180, &loop).Rising().IfHigh([sched, this]() {
    if (map.controllers.driver.GetRightBumper()) {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.outerGrid1)); // Outer Grid 1 (closest to centre)
    } else{
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.innerGrid3)); // Inner Grid 3 (closest to centre)
    }
  });
  // LEFT D-PAD
  map.controllers.driver.POV(270, &loop).Rising().IfHigh([sched, this]() {
    if (map.controllers.driver.GetRightBumper()) {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.centreGrid3)); // Community Grid 3 (outer grid side)
    } else {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.centreGrid1)); // Community Grid 1 (inner grid side)
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


  swerve->OnStart();
  
}

void Robot::TeleopPeriodic() { }

void Robot::DisabledInit() { }
void Robot::DisabledPeriodic() { }

void Robot::TestInit() { }
void Robot::TestPeriodic() { }

/* SIMULATION */

// #include <frc/simulation/BatterySim.h>
// #include <frc/simulation/RoboRioSim.h>
// #include <networktables/NetworkTableInstance.h>
// #include "ControlUtil.h"

// static units::second_t lastSimPeriodic{0};
// static auto simTable = nt::NetworkTableInstance::GetDefault().GetTable("/sim");

// struct SimConfig {
//   ::sim::ArmavatorSim arm;
//   wom::sim::SwerveDriveSim swerveSim; 
// };
// SimConfig *simConfig;

// void Robot::SimulationInit() {
//   simConfig = new SimConfig{
//     ::sim::ArmavatorSim(map.armavator.config),
//     wom::sim::SwerveDriveSim(map.swerveBase.config, 0.5 * 6_lb * 7.5_in * 7.5_in)
//   };

//   lastSimPeriodic = wom::now();
// }

// void Robot::SimulationPeriodic() {
//   auto dt = wom::now() - lastSimPeriodic;

//   simConfig->arm.OnUpdate(dt);
//   simConfig->swerveSim.Update(dt);

//   auto batteryVoltage = units::math::min(units::math::max(frc::sim::BatterySim::Calculate({
//     // simConfig->arm.GetCurrent()

//   }), 0_V), 12_V);
//   frc::sim::RoboRioSim::SetVInVoltage(batteryVoltage);
//   simTable->GetEntry("batteryVoltage").SetDouble(batteryVoltage.value()); 

//   lastSimPeriodic = wom::now();
// };