#include "Robot.h"
#include "behaviour/BehaviourScheduler.h"
#include "behaviour/Behaviour.h"
#include "behaviour/SwerveBaseBehaviour.h"
#include "behaviour/SideIntakeBehaviour.h"
#include "behaviour/GripperBehaviour.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/event/BooleanEvent.h>
#include <units/math.h>

using namespace frc;
using namespace behaviour;

static units::second_t lastPeriodic;

void Robot::RobotInit() {
  lastPeriodic = wom::now();

  vision = new Vision(map.vision.config);

  sideIntake = new SideIntake(map.sideIntake.config);
  BehaviourScheduler::GetInstance()->Register(sideIntake);
  sideIntake->SetDefaultBehaviour([this]() {
    return make<SideIntakeBehaviour>(sideIntake, map.controllers.codriver);
  });

  swerve = new wom::SwerveDrive(map.swerveBase.config, frc::Pose2d());
  BehaviourScheduler::GetInstance()->Register(swerve);
  swerve->SetDefaultBehaviour([this]() {
    return make<ManualDrivebase>(swerve, &map.controllers.driver);
  });

  //creates an instance of the armavator that can be used
  armavator = new Armavator(map.armavator.arm.gearbox, map.armavator.elevator.gearbox, map.armavator.config);
  BehaviourScheduler::GetInstance()->Register(armavator);
  armavator->SetDefaultBehaviour([this]() {
    //sets default behaviour class
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

  // map.swerveBase.turnMotors[0]->Set(map.controllers.driver.GetRightX());
  // map.swerveBase.driveMotors[0]->Set(map.controllers.driver.GetLeftY());
  
  // Gets each module's supply and output currents and outputs them onto networktables
  map.swerveTable.swerveDriveTable->GetEntry("Module1SupplyCurrent").SetDoubleArray(std::vector<double>({map.swerveBase.driveMotors[0]->GetSupplyCurrent(), map.swerveBase.turnMotors[0]->GetSupplyCurrent()}));
  map.swerveTable.swerveDriveTable->GetEntry("Module1OutputCurrent").SetDoubleArray(std::vector<double>({map.swerveBase.driveMotors[0]->GetOutputCurrent(), map.swerveBase.turnMotors[0]->GetOutputCurrent()}));
  map.swerveTable.swerveDriveTable->GetEntry("Module2SupplyCurrent").SetDoubleArray(std::vector<double>({map.swerveBase.driveMotors[1]->GetSupplyCurrent(), map.swerveBase.turnMotors[1]->GetSupplyCurrent()}));
  map.swerveTable.swerveDriveTable->GetEntry("Module2OutputCurrent").SetDoubleArray(std::vector<double>({map.swerveBase.driveMotors[1]->GetOutputCurrent(), map.swerveBase.turnMotors[1]->GetOutputCurrent()}));
  map.swerveTable.swerveDriveTable->GetEntry("Module3SupplyCurrent").SetDoubleArray(std::vector<double>({map.swerveBase.driveMotors[2]->GetSupplyCurrent(), map.swerveBase.turnMotors[2]->GetSupplyCurrent()}));
  map.swerveTable.swerveDriveTable->GetEntry("Module3OutputCurrent").SetDoubleArray(std::vector<double>({map.swerveBase.driveMotors[2]->GetOutputCurrent(), map.swerveBase.turnMotors[2]->GetOutputCurrent()}));
  map.swerveTable.swerveDriveTable->GetEntry("Module4SupplyCurrent").SetDoubleArray(std::vector<double>({map.swerveBase.driveMotors[3]->GetSupplyCurrent(), map.swerveBase.turnMotors[3]->GetSupplyCurrent()}));
  map.swerveTable.swerveDriveTable->GetEntry("Module4OutputCurrent").SetDoubleArray(std::vector<double>({map.swerveBase.driveMotors[3]->GetOutputCurrent(), map.swerveBase.turnMotors[3]->GetOutputCurrent()}));
  
  swerve->OnUpdate(dt);

  map.armTable.armManualTable->GetEntry("arm").SetDouble(map.armavator.arm.motor.GetSupplyCurrent());
  map.armTable.armManualTable->GetEntry("elv").SetDouble(map.armavator.elevator.motor.GetSupplyCurrent());
  armavator->OnUpdate(dt);

  map.intakeTable.intakeTable->GetEntry("state").SetString(sideIntake->GetState());

  sideIntake->OnUpdate(dt);

  gripper->OnUpdate(dt);
  // auto visionPose = vision->OnUpdate(dt);

  // if (visionPose.has_value()){
  //   swerve->AddVisionMeasurement(visionPose.value().first.ToPose2d(), visionPose.value().second);
  // }
}

void Robot::AutonomousInit() { }
void Robot::AutonomousPeriodic() { }

void Robot::TeleopInit() {
  loop.Clear();
  BehaviourScheduler *sched = BehaviourScheduler::GetInstance();

  swerve->OnStart();

  // Swervedrivebase grid poses
  map.controllers.driver.POV(0, &loop).Rising().IfHigh([sched, this]() { // up dpad
    if (map.controllers.driver.GetAButton()) {
      if (map.controllers.driver.GetYButton()){
        sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.centreGrid2)); // central grid
      } else {
        sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.outerGrid3)); // Outer Grid 3 (furthest from centre)
      }
    } else {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve,map.swerveGridPoses.innerGrid1)); // Inner Grid 1 (furthest from centre)
    }
  });
  map.controllers.driver.POV(90, &loop).Rising().IfHigh([sched, this]() { // right dpad
    if (map.controllers.driver.GetAButton()) {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.outerGrid2)); // Outer Grid 2
    } else {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.innerGrid2)); // Inner Grid 2
    }
  });
  map.controllers.driver.POV(180, &loop).Rising().IfHigh([sched, this]() { // down dpad
    if (map.controllers.driver.GetAButton()) {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.outerGrid1)); // Outer Grid 1 (closest to centre)
    } else{
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.innerGrid3)); // Inner Grid 3 (closest to centre)
    }
  });
  map.controllers.driver.POV(270, &loop).Rising().IfHigh([sched, this]() { // left dpad
    if (map.controllers.driver.GetAButton()) {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.centreGrid3)); // Community Grid 3 (outer grid side)
    } else {
      sched->Schedule(make<DrivebasePoseBehaviour>(swerve, map.swerveGridPoses.centreGrid1)); // Community Grid 1 (inner grid side)
    }
  });


  map.controllers.driver.X(&loop).Rising().IfHigh([sched, this]() {
    sched->Schedule(make<XDrivebase>(swerve));
  });

  // swerve->OnStart();



  // if (map.controllers.codriver.GetLeftY() <= 0.05 && map.controllers.codriver.GetLeftY() >= 0.05 && map.controllers.codriver.GetRightY() <= 0.05 && map.controllers.codriver.GetRightY() >= 0.05) {
  //   sched->Schedule(make<ArmavatorGoToPositionBehaviour>(armavator, ArmavatorPosition{armavator->_setpoint.height, armavator->_setpoint.angle}));
  // }

  // if(!map.controllers.codriver.GetAButton() && !map.controllers.codriver.GetBButton() && !map.controllers.codriver.GetXButton() && !map.controllers.codriver.GetYButton()) {
  // } else {
    //sets the premade positions usings buttonsS
    // map.controllers.codriver.A(&loop).Rising().IfHigh([sched, this]() {
    //   sched->Schedule(make<ArmavatorGoToPositionBehaviour>(armavator, ArmavatorPosition{1.0_m, 0_deg}));
    // });
    // map.controllers.codriver.B(&loop).Rising().IfHigh([sched, this]() {
    //   sched->Schedule(make<ArmavatorGoToPositionBehaviour>(armavator, ArmavatorPosition{1.2_m, -75_deg}));
    // });
    // map.controllers.codriver.X(&loop).Rising().IfHigh([sched, this]() {
    //   sched->Schedule(make<ArmavatorGoToPositionBehaviour>(armavator, ArmavatorPosition{1.0_m, 90_deg}));
    // });
    // map.controllers.codriver.Y(&loop).Rising().IfHigh([sched, this]() {
    //   sched->Schedule(make<ArmavatorGoToPositionBehaviour>(armavator, ArmavatorPosition{0.77_m, 45_deg}));
    // });
  // }

  // if(!map.controllers.codriver.GetAButton() && !map.controllers.codriver.GetBButton() && map.controllers.codriver.GetRightTriggerAxis() <= 0.05 && map.controllers.codriver.GetLeftTriggerAxis() <= 0.05) {
  //   map.armavator.arm.gearbox.transmission->SetVoltage(0_V);
  //   map.armavator.elevator.gearbox.transmission->SetVoltage(0_V);
  // } else{
  //   if(map.controllers.codriver.GetAButton()) {
  //     map.armavator.arm.gearbox.transmission->SetVoltage(13_V);
  //   } else if (map.controllers.codriver.GetBButton()) {
  //     map.armavator.arm.gearbox.transmission->SetVoltage(-13_V);
  //   }else if(map.controllers.codriver.GetRightTriggerAxis() > 0.05) {
  //     map.armavator.elevator.gearbox.transmission->SetVoltage(13_V * map.controllers.codriver.GetRightTriggerAxis());
  //   } else if (map.controllers.codriver.GetLeftTriggerAxis() > 0.05) {
  //     map.armavator.elevator.gearbox.transmission->SetVoltage(-13_V * map.controllers.codriver.GetLeftTriggerAxis() );
  //   }
  // }

  // map.controllers.driver.B(&loop).Rising().IfHigh([sched, this]() {
  //   swerve->GetActiveBehaviour()->Interrupt();
  // });
  // swerve->OnStart();

  // _armSetpoint = armavator->_setpoint.angle;
  // _elevatorSetpoint = armavator->_setpoint.height;

  // _armSetpoint = 60_deg;
  // _elevatorSetpoint = 0_m;

  // sched->Schedule(make<ArmavatorManualBehaviour>(armavator, map.controllers.codriver));

}

void Robot::TeleopPeriodic() {
  map.controlSystem.pcmCompressor.EnableDigital();
  // BehaviourScheduler *sched = BehaviourScheduler::GetInstance();
  // map.controlSystem.pcmCompressor.Enable();

  // bool enabled = map.controlSystem.pcmCompressor.Enabled();
  // bool pressureSwitch = map.controlSystem.pcmCompressor.GetPressureSwitchValue();
  // double current = map.controlSystem.pcmCompressor.GetCompressorCurrent();
  // units::meter_t addHeight = map.controllers.codriver.GetLeftY();

  // _armSetpoint += (map.controllers.codriver.GetLeftY() * 1_rad) / 100;
  // _elevatorSetpoint += (map.controllers.codriver.GetRightY() * 1_m) / 100;

  // sched->Schedule(make<ArmavatorGoToPositionBehaviour>(armavator, ArmavatorPosition{_elevatorSetpoint, _armSetpoint}));

  map.armTable.armManualTable->GetEntry("armSetpoint").SetDouble(_armSetpoint.convert<units::degree>().value());
  map.armTable.armManualTable->GetEntry("elevatorSetpoint").SetDouble(_elevatorSetpoint.convert<units::meter>().value());
}

void Robot::DisabledInit() { 
  // map.controlSystem.pcmCompressor.Disable();
}
void Robot::DisabledPeriodic() {}

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