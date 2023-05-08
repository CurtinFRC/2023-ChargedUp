#include "Auto.h"

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
  std::shared_ptr<Behaviour> midPathing;

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


  auto piece1_1 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.placeGrid1) | make<WaitTime>(6_s); // because PID
  
  auto piece2_1 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabReturnPos) | make<WaitTime>(6_s); // because PID
  auto piece2_2 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabChargestationAvoidPos) | make<WaitTime>(6_s); // because PID
  auto piece2_3 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.retrievePiece1Pos) | make<WaitTime>(6_s); // because PID
  auto piece2_4 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabChargestationAvoidPos) | make<WaitTime>(6_s); // because PID
  auto piece2_5 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabReturnPos) | make<WaitTime>(6_s); // because PID
  auto piece2_6 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.placeGrid2) | make<WaitTime>(6_s); // because PID
  
  auto piece3_1 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabReturnPos) | make<WaitTime>(6_s); // because PID
  auto piece3_2 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabChargestationAvoidPos) | make<WaitTime>(6_s); // because PID
  auto piece3_3 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.retrievePiece2Pos) | make<WaitTime>(6_s); // because PID
  auto piece3_4 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabChargestationAvoidPos) | make<WaitTime>(6_s); // because PID
  auto piece3_5 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabReturnPos) | make<WaitTime>(6_s); // because PID
  auto piece3_6 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.placeGrid3) | make<WaitTime>(6_s); // because PID
  
  auto piece4_1 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabReturnPos) | make<WaitTime>(6_s); // because PID
  auto piece4_2 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabChargestationAvoidPos) | make<WaitTime>(6_s); // because PID
  auto piece4_3 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.retrievePiece3Pos) | make<WaitTime>(6_s); // because PID
  auto piece4_4 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabChargestationAvoidPos) | make<WaitTime>(6_s); // because PID
  auto piece4_5 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabReturnPos) | make<WaitTime>(6_s); // because PID
  auto piece4_6 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.placeGrid4) | make<WaitTime>(6_s); // because PID
  
  auto piece5_1 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabReturnPos) | make<WaitTime>(6_s); // because PID
  auto piece5_2 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabChargestationAvoidPos) | make<WaitTime>(6_s); // because PID
  auto piece5_3 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.collectPath.retrievePiece3Pos) | make<WaitTime>(6_s); // because PID
  auto piece5_4 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabChargestationAvoidPos) | make<WaitTime>(6_s); // because PID
  auto piece5_5 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.grabReturnPos) | make<WaitTime>(6_s); // because PID
  auto piece5_6 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.pathingPoses.placeGrid5) | make<WaitTime>(6_s); // because PID
  
  midPathing = midPathing << piece1_1;
  if (calledFromID >= 2){  // if double
    midPathing = midPathing << piece2_1 << piece2_2 << piece2_3 << piece2_4 << piece2_5 << piece2_6;
  }
  if (calledFromID >= 3){  // if triple
    midPathing = midPathing << piece3_1 << piece3_2 << piece3_3 << piece3_4 << piece3_5 << piece3_6;
  }
  if (calledFromID >= 4){  // if quadruple
    midPathing = midPathing << piece4_1 << piece4_2 << piece4_3 << piece4_4 << piece4_5 << piece4_6;
  }
  if (calledFromID == 5){  // if quintuple
    midPathing = midPathing << piece5_1 << piece5_2 << piece5_3 << piece5_4 << piece5_5 << piece5_6;
  }
  autoPathingDetails.midPathing = midPathing;

  switch (endConfig) {
    case EndingConfig::Dock:
        {        
        auto wait_until1 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.dock_LineUp_Pos) | make<WaitTime>(3_s); // because PID
        auto wait_until2 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.dockPos) | make<WaitTime>(3_s); // because PID
        endPathing = midPathing << wait_until1 << wait_until2 << make<DrivebaseBalance>(drivebase.swerve, drivebase.gyro);
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

//Auto testing behaviour 
std::shared_ptr<behaviour::Behaviour> SubsystemTestPlace(Armavator *armavator) {
  return 
    make<WaitTime>(1_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.8_m, -40_deg)
    // << wait_until
    << make<WaitTime>(5_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.7_m, -10_deg)
    << make<WaitTime>(1_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.6_m, 0_deg);
}

//places low then balences 
std::shared_ptr<Behaviour> LowPlaceBalence(Drivebase drivebase, Armavator *armavator) {
  return 
  make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, -25_deg)->WithTimeout(2_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.82_m, -30_deg)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0.3_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{-0.4_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0.5_m, 0_m, 0_deg})->WithTimeout(2_s) //2.5
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 7_V)->Until(make<WaitFor>([drivebase]() {
      return units::math::abs(drivebase.gyro->GetPitch()) > 10_deg ||  units::math::abs(drivebase.gyro->GetRoll()) > 10_deg;
    }));
}

//these are all the auto's that were used at comp

//just drive forward and balence 
std::shared_ptr<Behaviour> Balence(Drivebase drivebase, Armavator *armavator) {
  return 
    make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, -50_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.8_m, 0_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.2_m, 40_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 90_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 90_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 120_deg) //arm must start back in order to get up the charge station 
    << make<WaitTime>(3_s)
    //moves forwards until a tilt of more than 10 degrees is detected, then moves onto the next behaviour 
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([drivebase]() {
      return units::math::abs(drivebase.gyro->GetPitch()) > 10_deg ||  units::math::abs(drivebase.gyro->GetRoll()) > 10_deg;
    }))
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 70_deg) //arm must swing over to get up the charge station 
    << make<DrivebaseBalance>(drivebase.swerve, drivebase.gyro); //automatically balences the robot 
}

//Places cone on high then drives forwards to balence, has a couple of weird timeout things that were used instead of tuning PID better as this was quicker to do at comp
std::shared_ptr<Behaviour> PlaceBalence(Drivebase drivebase, Armavator *armavator, Gripper *gripper) {
  return 
    make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, -55_deg)->WithTimeout(1_s) //move out of starting config 
      << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{-0.15_m, 0_m, 0_deg})->WithTimeout(0.3_s)
      << make<ArmavatorGoToAutoSetpoint>(armavator, 0.8_m, 0_deg)
      << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 90_deg)->WithTimeout(1_s)
      << make<ArmavatorGoToAutoSetpoint>(armavator, 0.4_m, 140_deg, 0.4, 0.15)->WithTimeout(1.2_s)
      << make<WaitTime>(0.5_s)
      << make<ArmavatorGoToAutoSetpoint>(armavator, 0.45_m, 160_deg, 0.2, 0.2)->WithTimeout(1.2_s)
      << make<GripperAutoBehaviour>(gripper, 1)->Until(make<WaitTime>(1_s))
      << make<GripperAutoBehaviour>(gripper, 3)->Until(make<WaitTime>(1_s))
      << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0.5_m, 0_m, 0_deg}, 7_V)->WithTimeout(1_s)
      << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 120_deg)
      //moves forwards until a tilt of more than 10 degrees is detected, then moves onto the next behaviour 
      << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{4_m, 0_m,  0_deg}, 4_V)->Until(make<WaitFor>([drivebase]() {
        return units::math::abs(drivebase.gyro->GetPitch()) > 10_deg ||  units::math::abs(drivebase.gyro->GetRoll()) > 10_deg;
      }))
      << make<ArmavatorGoToAutoSetpoint>(armavator, 0.1_m, 70_deg)
      << make<DrivebaseBalance>(drivebase.swerve, drivebase.gyro);
}

//Single place and possible mobility 
std::shared_ptr<Behaviour> HighPlace(Drivebase drivebase, Armavator *armavator, Gripper *gripper){
  return 
    make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, -55_deg)->WithTimeout(1_s) //start in starting config
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0.1_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, 0_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.3_m, 90_deg)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.4_m, 140_deg, 0.4, 0.15)
    << make<WaitTime>(0.5_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.25_m, 160_deg, 0.2, 0.2)
    << make<GripperAutoBehaviour>(gripper, 1)->Until(make<WaitTime>(1_s))
    << make<GripperAutoBehaviour>(gripper, 3)->Until(make<WaitTime>(1_s))
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0.3_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<ArmavatorGoToAutoSetpoint>(armavator, 0.3_m, 90_deg)
    // << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{4_m, 0_m, 0_deg})->WithTimeout(5_s) //used if you also want mobility 
    << make<WaitTime>(10_s);
}

//mobility, drop cube in low goal 
std::shared_ptr<Behaviour> ForwardDrive(Drivebase drivebase, Armavator *armavator){
  return
    // make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{-0.4_m, 0_m, 0_deg})->WithTimeout(1_s)
    make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, -25_deg)->WithTimeout(2_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.82_m, -30_deg)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0.3_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{-0.4_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{3.5_m, 0_m, 0_deg})->WithTimeout(3_s) //2.5
    << make<WaitTime>(20_s);
}

//mobility, drop cube in low goal 
std::shared_ptr<Behaviour> LowPlace(Drivebase drivebase, Armavator *armavator){
  return
    // make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{-0.4_m, 0_m, 0_deg})->WithTimeout(1_s)
    make<ArmavatorGoToAutoSetpoint>(armavator, 0.9_m, -25_deg)->WithTimeout(2_s)
    // << make<ArmavatorGoToAutoSetpoint>(armavator, 0.82_m, -30_deg)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0.3_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{-0.4_m, 0_m, 0_deg})->WithTimeout(1_s)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0.5_m, 0_m, 0_deg})->WithTimeout(2_s) //2.5
    << make<WaitTime>(20_s);
}