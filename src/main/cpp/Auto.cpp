#include "Auto.h"
// #include "Poses.h"

#include "behaviour/SwerveBaseBehaviour.h"
#include "behaviour/ArmavatorBehaviour.h"
#include "behaviour/GripperBehaviour.h"

using namespace behaviour;

DefinedPoses definedPoses;


/*
for adding auto behaviours for other systems, add into the autoPathingDetails.endPathing, and between current behaviours
*/

AutoPathDetails GetAutoPathingDetails(Drivebase drivebase, StartingConfig startConfig, EndingConfig endConfig, bool blueAlliance, int calledFromID, std::vector<frc::Pose2d> adjustmentPoses){
  AutoPathDetails autoPathingDetails;
  std::shared_ptr<Behaviour> adjustmentPathing;
  std::shared_ptr<Behaviour> endPathing;

  autoPathingDetails.endPathing = make<WaitTime>(0_ms);
  for (frc::Pose2d pose : adjustmentPoses) {
    adjustmentPathing = autoPathingDetails.endPathing << make<DrivebasePoseBehaviour>(drivebase.swerve, pose);
    autoPathingDetails.endPathing = adjustmentPathing;
  };
  if (blueAlliance){  definedPoses.alliancePoses = definedPoses.blueAlliancePoses;  }
  else{  definedPoses.alliancePoses = definedPoses.redAlliancePoses;  }

  switch (startConfig){
    case StartingConfig::Top:
      definedPoses.poseSet = definedPoses.alliancePoses.topPoses;
     break;
    case StartingConfig::Middle:
      definedPoses.poseSet = definedPoses.alliancePoses.middlePoses;
     break;
    case StartingConfig::Bottom:
      definedPoses.poseSet = definedPoses.alliancePoses.bottomPoses;
     break;
  }

  autoPathingDetails.startPos = definedPoses.poseSet.startPos;

  switch (endConfig) {
    case EndingConfig::Dock:
        {
        auto wait_until1 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.dock_LineUp_Pos) | make<WaitTime>(3_s); // because PID
        auto wait_until2 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.dockPos) | make<WaitTime>(3_s); // because PID
        endPathing = wait_until1 << wait_until2 << make<DrivebaseBalance>(drivebase.swerve, drivebase.gyro);
      break;}
    case EndingConfig::PrepareManual:
        endPathing = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.subStationWaitPos);
      break;
    case EndingConfig::Collect:
        {// The Collect assumes that we start with a held game piece, that held game piece will be the only one for SINGLE, hence no swerve motion will take place
        if (calledFromID == 1){   break;   }   // If SINGLE, do nothing
        auto drive1 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.startingFromPos) | make<WaitTime>(6_s); // because PID;
        auto drive2 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.retrievePiece1Pos) | make<WaitTime>(6_s); // because PID;
        auto drive3 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.returnPiece1Pos) | make<WaitTime>(6_s); // because PID;
        auto drive4 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.retrievePiece2Pos) | make<WaitTime>(6_s); // because PID;
        auto drive5 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.returnPiece2Pos) | make<WaitTime>(6_s); // because PID;
        auto drive6 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.retrievePiece3Pos) | make<WaitTime>(6_s); // because PID;
        auto drive7 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.returnPiece3Pos) | make<WaitTime>(6_s); // because PID;
        auto drive8 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.retrievePiece4Pos) | make<WaitTime>(6_s); // because PID;
        auto drive9 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.returnPiece4Pos) | make<WaitTime>(6_s); // because PID;
        if (calledFromID == 2){  // if double
          endPathing = drive1 << drive2 << drive3;
        }
        else if (calledFromID == 3){  // if triple
          endPathing = drive1 << drive2 << drive3 << drive4 << drive5;
        }
        else if (calledFromID == 4){  // if quadruple
          endPathing = drive1 << drive2 << drive3 << drive4 << drive5 << drive6 << drive7;
        }
        else if (calledFromID == 5){  // if quintuple
          endPathing = drive1 << drive2 << drive3 << drive4 << drive5 << drive6 << drive7 << drive8 << drive9;
        }}
      break;
    case EndingConfig::Taxi:
        endPathing = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.taxiPos);
      break;
    case EndingConfig::Steal:
        endPathing = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.stealPos);
      break;
  }

  autoPathingDetails.endPathing = autoPathingDetails.endPathing << endPathing;
  return autoPathingDetails;
}


std::shared_ptr<Behaviour> DockBot(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  AutoPathDetails autoPathDetails = GetAutoPathingDetails(drivebase, startConfig, endConfig, blueAlliance, 0);
  return
    make<DrivebasePoseBehaviour>(drivebase.swerve, autoPathDetails.startPos)
    << autoPathDetails.endPathing;
}

std::shared_ptr<Behaviour> ForwardDrive(Drivebase drivebase, Armavator *armavator){
  return
    // make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{-0.4_m, 0_m, 0_deg})->WithTimeout(1_s)
    make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, -25_deg)->WithTimeout(2_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.82_m, -30_deg)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0.3_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{-0.4_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{3.5_m, 0_m, 0_deg})->WithTimeout(3_s) //2.5
    << make<WaitTime>(20_s);
    // make<ArmavatorGoToAutoSetpoint>(armavator, 0.5_m, 40_deg)
    
}

std::shared_ptr<Behaviour> Balence(Drivebase drivebase, Armavator *armavator) {
  return 
    make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 90_deg)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 7_V)->Until(make<WaitFor>([drivebase]() {
      return units::math::abs(drivebase.gyro->GetPitch()) > 10_deg ||  units::math::abs(drivebase.gyro->GetRoll()) > 10_deg;
    }));
}

std::shared_ptr<Behaviour> Single(Drivebase drivebase, Armavator *armavator, Gripper *gripper, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  AutoPathDetails autoPathDetails = GetAutoPathingDetails(drivebase, startConfig, endConfig, blueAlliance, 1);
  return 
    make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, -55_deg)->WithTimeout(1_s) //start in starting config
    // << make<WaitTime>(0.01_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, 0_deg)->WithTimeout(2_s)
    // << make<WaitTime>(0.01_s)
    // << ((
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0.1_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, 0_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.3_m, 90_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.4_m, 140_deg, 0.4, 0.15)
    << make<WaitTime>(0.5_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.25_m, 160_deg, 0.2, 0.2)
    // ) | make<GripperAutoBehaviour>(gripper, 2))
    << make<GripperAutoBehaviour>(gripper, 1)->Until(make<WaitTime>(1_s))
    << make<GripperAutoBehaviour>(gripper, 3)->Until(make<WaitTime>(1_s))
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0.3_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.3_m, 90_deg)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{4_m, 0_m, 0_deg})->WithTimeout(5_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.5_m, 90_deg, 0.5, 0.15)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.7_m, 120_deg, 0.5, 0.15)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.5_m, 140_deg, 0.5, 0.15)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.7_m, 120_deg, 0.5, 0.2)
    // << make<WaitTime>(0.01_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 120_deg)
    // << make<WaitTime>(0.01_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.6_m, 130_deg)
    << make<WaitTime>(10_s);
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0_m, 110_deg)
    // // << make<WaitTime>(0.01_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.2_m, 120_deg)
    // // << make<WaitTime>(0.01_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.6_m, 140_deg)
    // // << make<WaitTime>(0.01_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.6_m, 160_deg)
    // // << make<WaitTime>(0.01_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.20_m, 160_deg)

    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.2_m, 130_deg)->WithTimeout(4_s)


    // // << make<WaitTime>(0.01_s)
    // << make<GripperAutoBehaviour>(gripper, 1)->Until(make<WaitTime>(1_s))
    // << make<GripperAutoBehaviour>(gripper, 3)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.20_m, 120_deg)
    // // << make<WaitTime>(0.01_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 90_deg)
    // // << make<WaitTime>(0.01_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.0_m, 70_deg);
    // << make<DrivebasePoseBehaviour>(drivebase.swerve, autoPathDetails.startPos)
    // << autoPathDetails.endPathing;
}




//Just drive auto 
// std::shared_ptr<Behaviour> Single(Drivebase drivebase, Armavator *armavator, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
//   AutoPathDetails autoPathDetails = GetAutoPathingDetails(drivebase, startConfig, endConfig, blueAlliance, 1);
//   return 
//     make<DrivebasePoseBehaviour>(drivebase.swerve, autoPathDetails.startPos)
//     << autoPathDetails.endPathing
//     << make<WaitTime>(1_s)
//     << make<ArmavatorGoToAutoSetpoint>(armavator, 0.3_m, 60_deg);
// }

std::shared_ptr<Behaviour> Double(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  AutoPathDetails autoPathDetails = GetAutoPathingDetails(drivebase, startConfig, endConfig, blueAlliance, 2);
  auto drive1 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.startingFromPos) | make<WaitTime>(6_s); // because PID;
  auto drive2 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.retrievePiece1Pos) | make<WaitTime>(6_s); // because PID;

  return
    make<DrivebasePoseBehaviour>(drivebase.swerve, autoPathDetails.startPos)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.retrievePiece1Pos)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.alliancePoses.gridPoses.outerGrid3)
    << autoPathDetails.endPathing;
}

std::shared_ptr<Behaviour> Triple(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  AutoPathDetails autoPathDetails = GetAutoPathingDetails(drivebase, startConfig, endConfig, blueAlliance, 3);

  // this function is currently built for testing while starting at (0, 0) due to lack of vision
  auto wait_until = make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{224_in, -45_in, 0_deg}) | make<WaitTime>(3_s);
  auto wait_until2 = make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0_in, 1.5_m, 0_deg}) | make<WaitTime>(2_s); 
    
  return
    make<DrivebasePoseBehaviour>(drivebase.swerve, autoPathDetails.startPos)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{224_in, 0_m, 0_deg})
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0_m, 0_m, 0_deg})

    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{145_in, 0_m, 0_deg})
    << wait_until
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{145_in, 0_in, 0_deg})
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0_in, 0_in, 0_deg})

    << autoPathDetails.endPathing;
}

std::shared_ptr<Behaviour> Quad(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  AutoPathDetails autoPathDetails = GetAutoPathingDetails(drivebase, startConfig, endConfig, blueAlliance, 4);
  return
    make<DrivebasePoseBehaviour>(drivebase.swerve, autoPathDetails.startPos)
    // << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{224_in, 0_m, 0_deg})
    // << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0_m, 0_m, 0_deg})
    << autoPathDetails.endPathing;
}

std::shared_ptr<Behaviour> Quintuple(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  AutoPathDetails autoPathDetails = GetAutoPathingDetails(drivebase, startConfig, endConfig, blueAlliance, 5);
  return
    make<DrivebasePoseBehaviour>(drivebase.swerve, autoPathDetails.startPos)
    // << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{224_in, 0_m, 0_deg})
    // << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0_m, 0_m, 0_deg})
    << autoPathDetails.endPathing;
}

std::shared_ptr<behaviour::Behaviour> SubsystemTestPlace(Armavator *armavator) {
  return 
    make<WaitTime>(1_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.8_m, -40_deg)
    // << wait_until
    << make<WaitTime>(5_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.7_m, -10_deg)
    << make<WaitTime>(1_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.6_m, 0_deg);
    // << make<ArmavatorGoToPositionBehaviour(armavator, pos);
    // << make<ArmavatorGoToAutoSetpoint(armavator)
    // << make<WaitTime>(0.3_s);
    // << make<ArmavatorGoToPositionBehaviour(armavator, {0_deg, 0.8_m})
    // << make<WaitTime>(0.3_s)
    // << make<ArmavatorGoToPositionBehaviour(armavator, {90_deg, 0.3_m})
    // << make<WaitTime>(0.3_s)
    // << make<ArmavatorGoToPositionBehaviour(armavator, {160_deg, 0.4_m});
}