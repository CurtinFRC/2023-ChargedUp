#include "Auto.h"
// #include "Poses.h"

#include "behaviour/SwerveBaseBehaviour.h"

using namespace behaviour;

DefinedPoses definedPoses;

AutoPathDetails GetAutoPathingDetails(Drivebase drivebase, StartingConfig startConfig, EndingConfig endConfig, bool blueAlliance, int calledFromID, std::vector<frc::Pose2d> adjustmentPoses){
  AutoPathDetails autoPathingDetails;
  std::shared_ptr<Behaviour> adjustmentPathing;
  std::shared_ptr<Behaviour> endPathing;

  autoPathingDetails.endPathing = make<WaitTime>(0_ms);
  for (frc::Pose2d pose : adjustmentPoses) {
    adjustmentPathing = autoPathingDetails.endPathing << make<DrivebasePoseBehaviour>(drivebase.swerve, pose);
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
        auto wait_until1 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.dock_LineUp_Pos) | make<WaitTime>(3_s); // because PID
        auto wait_until2 = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.dockPos) | make<WaitTime>(2_s); // because PID
        endPathing = wait_until1 << wait_until2 << make<DrivebaseBalance>(drivebase.swerve, drivebase.gyro);
      break;}
    case EndingConfig::PrepareManual:
        endPathing = make<DrivebasePoseBehaviour>(drivebase.swerve, definedPoses.poseSet.subStationWaitPos);
      break;
    case EndingConfig::Collect:
        /* have fun with this one Liam :D */
        if (calledFromID == 1){   break;   }   // If SINGLE, do nothing
        /*
        if (id = none):
        if (id = single):
        if (id = double):
        if (id = triple):
        if (id = quad):
        if (id = quintuple):
        */
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

std::shared_ptr<Behaviour> Single(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  AutoPathDetails autoPathDetails = GetAutoPathingDetails(drivebase, startConfig, endConfig, blueAlliance, 1);
  return 
    make<DrivebasePoseBehaviour>(drivebase.swerve, autoPathDetails.startPos)
    // place gamepiece
    << autoPathDetails.endPathing;

}

std::shared_ptr<Behaviour> Double(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig){
  AutoPathDetails autoPathDetails = GetAutoPathingDetails(drivebase, startConfig, endConfig, blueAlliance, 2);
  return
    make<DrivebasePoseBehaviour>(drivebase.swerve, autoPathDetails.startPos)
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{224_in, 0_m, 0_deg})
    << make<DrivebasePoseBehaviour>(drivebase.swerve, frc::Pose2d{0_m, 0_m, 0_deg})
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
  return make<WaitTime>(1_s);
}
