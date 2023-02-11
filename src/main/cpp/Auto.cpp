#include "Auto.h"
#include "Poses.h"

#include "behaviour/SwerveBaseBehaviour.h"
#include "behaviour/ArmavatorBehaviour.h"

using namespace behaviour;

DefinedPoses definedPoses;

std::shared_ptr<Behaviour> Drive(wom::SwerveDrive *swerve, wom::NavX *gyro){
  return
  make<WaitTime>(1_s)
  << make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{2_m, 0_m, 0_deg})
  << make<DrivebaseBalance>(swerve, gyro);
}


AutoPathDetails GetAutoPathingDetails(SwervePack swerve, StartingConfig startConfig, EndingConfig endConfig, bool blueAlliance, std::vector<frc::Pose2d> adjustmentPoses){
  AutoPathDetails autoPathingDetails;
  std::shared_ptr<Behaviour> adjustmentPathing;
  std::shared_ptr<Behaviour> endPathing;

  autoPathingDetails.endPathing = make<WaitTime>(0_ms);
  for (frc::Pose2d pose : adjustmentPoses) {
    adjustmentPathing = autoPathingDetails.endPathing << make<DrivebasePoseBehaviour>(swerve.swerve, pose);
    autoPathingDetails.endPathing = adjustmentPathing;
  };

  switch (startConfig){
    case StartingConfig::Top:
      if (blueAlliance){  definedPoses.poseSet = definedPoses.top_Blue;  break;  }
      definedPoses.poseSet = definedPoses.top_Red;  break;
    case StartingConfig::Middle:
      if (blueAlliance){  definedPoses.poseSet = definedPoses.middle_Blue;  break;  }
      definedPoses.poseSet = definedPoses.middle_Red;  break;
    case StartingConfig::Bottom:
      if (blueAlliance){  definedPoses.poseSet = definedPoses.bottom_Blue;  break;  }
      definedPoses.poseSet = definedPoses.bottom_Red;  break;
  }
  autoPathingDetails.startPos = definedPoses.poseSet.startPos;

  switch (endConfig) {
    case EndingConfig::Dock:
        {
        auto wait_until = make<DrivebasePoseBehaviour>(swerve.swerve, definedPoses.poseSet.dock_LineUp_Pos) | make<WaitTime>(3_s); // because PID
        auto wait_until2 =make<DrivebasePoseBehaviour>(swerve.swerve, definedPoses.poseSet.dockPos) | make<WaitTime>(2_s); // because PID
        endPathing = //autoPathingDetails.endPathing 
        wait_until
        << wait_until2
        << make<DrivebaseBalance>(swerve.swerve, swerve.gyro);
      break;}
    case EndingConfig::PrepareManual:
        endPathing = //autoPathingDetails.endPathing
        make<DrivebasePoseBehaviour>(swerve.swerve, definedPoses.poseSet.subStationWaitPos);
      break;
    case EndingConfig::Collect:
          /* haha have fun with this Liam :D */
      break;
    case EndingConfig::Taxi:
        endPathing = //autoPathingDetails.endPathing
        make<DrivebasePoseBehaviour>(swerve.swerve, definedPoses.poseSet.taxiPos);
      break;
    case EndingConfig::Steal:
        endPathing = //autoPathingDetails.endPathing
        make<DrivebasePoseBehaviour>(swerve.swerve, definedPoses.poseSet.stealPos);
      break;
  }
  autoPathingDetails.endPathing = autoPathingDetails.endPathing << endPathing;
  return autoPathingDetails;
}


std::shared_ptr<Behaviour> DockBot(SwervePack swerve, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  return
  make<DrivebasePoseBehaviour>(swerve.swerve, frc::Pose2d{2_m, 0_m, 0_deg});
  // << make<DrivebaseBalance>(swerve, gyro);
}

std::shared_ptr<Behaviour> Single(SwervePack swerve, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  /*
  drive to start pos
  place owned game piece
  drive to meet ending point criteria
  (at some point arm would need to be repositioned into movement thing)
  */
  AutoPathDetails autoPathDetails = GetAutoPathingDetails(swerve, startConfig, endConfig, blueAlliance);
  return 
    make<DrivebasePoseBehaviour>(swerve.swerve, autoPathDetails.startPos)
    // place gamepiece
    << autoPathDetails.endPathing;

}

std::shared_ptr<Behaviour> Double(SwervePack swerve, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  AutoPathDetails autoPathDetails = GetAutoPathingDetails(swerve, startConfig, endConfig, blueAlliance);
  return
    make<DrivebasePoseBehaviour>(swerve.swerve, autoPathDetails.startPos)
    << make<DrivebasePoseBehaviour>(swerve.swerve, frc::Pose2d{224_in, 0_m, 0_deg})
    << make<DrivebasePoseBehaviour>(swerve.swerve, frc::Pose2d{0_m, 0_m, 0_deg})
    << autoPathDetails.endPathing;
}

std::shared_ptr<Behaviour> Triple(SwervePack swerve, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  AutoPathDetails autoPathDetails = GetAutoPathingDetails(swerve, startConfig, endConfig, blueAlliance);
  auto wait_until = make<DrivebasePoseBehaviour>(swerve.swerve, frc::Pose2d{224_in, -45_in, 0_deg}) | make<WaitTime>(3_s);
  auto wait_until2 = make<DrivebasePoseBehaviour>(swerve.swerve, frc::Pose2d{0_in, 1.5_m, 0_deg}) | make<WaitTime>(2_s); 
    
  return
    make<DrivebasePoseBehaviour>(swerve.swerve, autoPathDetails.startPos)
    << make<DrivebasePoseBehaviour>(swerve.swerve, frc::Pose2d{224_in, 0_m, 0_deg})
    << make<DrivebasePoseBehaviour>(swerve.swerve, frc::Pose2d{0_m, 0_m, 0_deg})

    << make<DrivebasePoseBehaviour>(swerve.swerve, frc::Pose2d{145_in, 0_m, 0_deg})
    << wait_until
    << make<DrivebasePoseBehaviour>(swerve.swerve, frc::Pose2d{145_in, 0_in, 0_deg})
    << make<DrivebasePoseBehaviour>(swerve.swerve, frc::Pose2d{0_in, 0_in, 0_deg})

    << autoPathDetails.endPathing;
}

std::shared_ptr<Behaviour> Quad(SwervePack swerve, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  return make<WaitTime>(1_s);
}





// Triple Score                 <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> BLUE_Top_Triple(wom::SwerveDrive *swerve){
    /*
    make<ArmavatorScoreHigh>();
    make<ArmavatorGoToBalancePosition() & drive to centreTopMidGamePiece & deploy intake
    after a certain amount of driving time, start intaking
    drive to adjacent inner grid & "(maybe)retract intake" & start moving arm up
    */

    auto wait_until = make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{224_in, -45_in, 0_deg}) | make<WaitTime>(4_s);
    auto wait_until2 = make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{0_in, 1.5_m, 0_deg}) | make<WaitTime>(2_s); 
    return
    make<WaitTime>(1_s)
    << make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{224_in, 0_m, 0_deg})
    << make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{0_m, 0_m, 0_deg})

    << make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{145_in, 0_m, 0_deg})
    << wait_until
    << make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{145_in, 0_in, 0_deg})
    << make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{0_in, 0_in, 0_deg});

    // << make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{0_in, -1_m, 0_deg})
    // << wait_until2
    // << make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{1_m, -1_m, 0_deg});
}












/*
FOR docking:
    drive to one of 3 points,
    drive towards centre a bit,
    activate balance behaviour

FOR collecting middle piece, starting from a grid position:
    drive to middle piece position (might require a straight movement, and then a diagonal, dodging the chargestation)
        after driving a certain distance (or time) engage intake
            after driving a certain distance (or time) activate intake

FOR returning to a grid position after collecting from centre:
    drive to grid position (might require a diagonal, straight then diagonal movement, dodging the chargestation)
        deploy gripper to grab intaked game object
            after some time, move up, to avoid intake
        after a small amount of time disable intake
        (MAYBE) after a small amount of time retract intake

        

*/