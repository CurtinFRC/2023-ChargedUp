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

  std::optional<units::meter_t> distance = map.gripper.gamepiecePresence.GetDistance();
  if (distance.has_value())
    nt::NetworkTableInstance::GetDefault().GetTable("TOF")->GetEntry("distance").SetDouble(distance.value().value());
  else
    nt::NetworkTableInstance::GetDefault().GetTable("TOF")->GetEntry("distance").SetDouble(-1);
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

  // map.controllers.driver.B(&loop).Rising().IfHigh([sched, this]() {
  //   swerve->GetActiveBehaviour()->Interrupt();
  //   map.swerveTable.swerveDriveTable->GetEntry("IsX-ed").SetBoolean(false);
  // });

  map.controllers.driver.A(&loop).Rising().IfHigh([sched, this]() {
    
    sched->Schedule(make<DrivebaseBalance>(swerve, &map.swerveBase.gyro));
  });

  // Current Keybinds:
  /*
    A - Balance Behaviour
    B - Interrupts any behaviour, and sets active behaviour to the manual drive behaviour
    X - X Wheels
    Y - Field Orientated / Robot Relative Toggle
    RightBumper - Acting as a shift key for grid poses
    LeftBumper - Acting as a shift key for grid poses
  */

  swerve->OnStart();
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

  double speed = map.controllers.codriver.GetRightY();
  // map.grTest.gripper.Set(speed);
  map.grTest.gripper.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,speed);


  map.armTable.armManualTable->GetEntry("armSetpoint").SetDouble(_armSetpoint.convert<units::degree>().value());
  map.armTable.armManualTable->GetEntry("elevatorSetpoint").SetDouble(_elevatorSetpoint.convert<units::meter>().value());
}

void Robot::DisabledInit() { 
  // map.controlSystem.pcmCompressor.Disable();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() { }
void Robot::TestPeriodic() { }
