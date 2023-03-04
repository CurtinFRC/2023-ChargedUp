#pragma once

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"


class DefinedPoses {
 public:
   struct CollectPath {
    frc::Pose2d startingFromPos;
    frc::Pose2d retrievePiece1Pos;
    frc::Pose2d returnPiece1Pos;
    frc::Pose2d retrievePiece2Pos;
    frc::Pose2d returnPiece2Pos;
    frc::Pose2d retrievePiece3Pos;
    frc::Pose2d returnPiece3Pos;
    frc::Pose2d retrievePiece4Pos;
    frc::Pose2d returnPiece4Pos;
  };

  struct Poses {
    frc::Pose2d startPos;
    frc::Pose2d dock_LineUp_Pos;
    frc::Pose2d dockPos;
    frc::Pose2d stealPos;
    frc::Pose2d taxiPos;
    frc::Pose2d subStationWaitPos;
    CollectPath collectPath;
  };
  Poses poseSet{};

  struct GamePiecePoses {
    frc::Pose2d gamePiece1Pos;  // the bottom-most game piece
    frc::Pose2d gamePiece2Pos;  // the bottom-middle game piece
    frc::Pose2d gamePiece3Pos;  // the top-middle game piece
    frc::Pose2d gamePiece4Pos;  // the top-most game piece
  };

  struct AlliancePoses {
    Poses topPoses{};
    Poses middlePoses{};
    Poses bottomPoses{};
    GamePiecePoses gamePiecePoses{};
  };


  AlliancePoses blueAlliancePoses{
    {  // Top Poses - START  */
      {72.055_in, 196.3_in, 0_deg}, // startPos
      {83.607_in, 139.388_in, 0_deg}, // dock_LineUp_Pos
      {122.808_in, 139.286_in, 0_deg}, // dockPos
      {287.887_in, 180.555_in, 0_deg}, // stealPos
      {171.061_in, 195.748_in, 0_deg}, // taxiPos
      {291.192_in, 246.31_in, 0_deg},  // subStationWaitPos
      {  /*  Collect Path - START  */
        {72.055_in, 196.3_in, 0_deg}, // startingFromPos
        {278.389_in, 180.185_in, 0_deg}, // retrievePiece1Pos
        {132.81_in, 156.185_in, 0_deg}, // returnPiece1Pos
        {278.389_in, 132.185_in, 0_deg}, // retrievePiece2Pos
        {132.81_in, 108.185_in, 0_deg}, // returnPiece2Pos
        {278.389_in, 84.185_in, 0_deg}, // retrievePiece3Pos
        {132.81_in, 60.185_in, 0_deg}, // returnPiece3Pos
        {278.389_in, 36.185_in, 0_deg}, // retrievePiece4Pos
        {132.81_in, 36.185_in, 0_deg} // returnPiece4Pos
      }  /*  Collect Path - END  */
    }, /*  Top Poses - END  */
    {  /*  Middle Poses - START  */
      {72.043_in, 86.608_in, 0_deg}, // startPos
      {83.629_in, 107.795_in, 0_deg}, // dock_LineUp_Pos
      {122.407_in, 107.992_in, 0_deg}, // dockPos

      {0_m, 0_m, 0_deg}, // stealPos
      {0_m, 0_m, 0_deg}, // taxiPos
      {0_m, 0_m, 0_deg},  // subStationWaitPos

    }, /*  Middle Poses - END  */
    {  /*  Bottom Poses - START  */
      // {72.061_in, 20.208_in, 0_deg}, // startPos
      // {82.138_in, 76.783_in, 0_deg}, // dock_LineUp_Pos
      // {124.158_in, 75.594_in, 0_deg}, // dockPos
      // {288.932_in, 35.832_in, 0_deg}, // stealPos
      // {221.744_in, 17.986_in, 0_deg}, // taxiPos


      // the following are just for testing without vision
      {0_in, 0_in, 0_deg}, // startPos
      {10.077_in, 60.575_in, 0_deg}, // dock_LineUp_Pos
      {52.097_in, 60.575_in, 0_deg}, // dockPos
      {288.932_in, 35.832_in, 0_deg}, // stealPos
      {221.744_in, 17.986_in, 0_deg}, // taxiPos

      {0_m, 0_m, 0_deg},  // subStationWaitPos
      {  /*  Collect Path - START  */
        {72.061_in, 20.208_in, 0_deg}, // startingFromPos
        {278.389_in, 36.185_in, 0_deg}, // retrievePiece1Pos
        {132.81_in, 60.185_in, 0_deg}, // returnPiece1Pos
        {278.389_in, 84.185_in, 0_deg}, // retrievePiece2Pos
        {132.81_in, 108.185_in, 0_deg}, // returnPiece2Pos
        {278.389_in, 132.185_in, 0_deg}, // retrievePiece3Pos
        {132.81_in, 156.185_in, 0_deg}, // returnPiece3Pos
        {278.389_in, 180.185_in, 0_deg}, // retrievePiece4Pos
        {132.81_in, 180.185_in, 0_deg} // returnPiece4Pos
      }  /*  Collect Path - END  */
    }, /*  Bottom Poses - END  */
    {  /*  Game Piece Poses - START  */
      {278.389_in, 36.185_in, 0_deg},  // the bottom-most game piece
      {278.389_in, 84.185_in, 0_deg},  // the bottom-middle game piece
      {278.389_in, 132.185_in, 0_deg},  // the top-middle game piece
      {278.389_in, 180.185_in, 0_deg}  // the top-most game piece
    }  /* Game Piece Poses - END  */
  };

  AlliancePoses redAlliancePoses{
    {  // Top Poses - START  */
      {72.055_in, 196.3_in, 0_deg}, // startPos
      {83.607_in, 139.388_in, 0_deg}, // dock_LineUp_Pos
      {122.808_in, 139.286_in, 0_deg}, // dockPos
      {287.887_in, 180.555_in, 0_deg}, // stealPos
      {171.061_in, 195.748_in, 0_deg}, // taxiPos
      {291.192_in, 246.31_in, 0_deg},  // subStationWaitPos
      {  /*  Collect Path - START  */
        {72.055_in, 196.3_in, 0_deg}, // startingFromPos
        {278.389_in, 180.185_in, 0_deg}, // retrievePiece1Pos
        {132.81_in, 156.185_in, 0_deg}, // returnPiece1Pos
        {278.389_in, 132.185_in, 0_deg}, // retrievePiece2Pos
        {132.81_in, 108.185_in, 0_deg}, // returnPiece2Pos
        {278.389_in, 84.185_in, 0_deg}, // retrievePiece3Pos
        {132.81_in, 60.185_in, 0_deg}, // returnPiece3Pos
        {278.389_in, 36.185_in, 0_deg}, // retrievePiece4Pos
        {132.81_in, 36.185_in, 0_deg} // returnPiece4Pos
      }  /*  Collect Path - END  */
    }, /*  Top Poses - END  */
    {  /*  Middle Poses - START  */
      {72.043_in, 86.608_in, 0_deg}, // startPos
      {83.629_in, 107.795_in, 0_deg}, // dock_LineUp_Pos
      {122.407_in, 107.992_in, 0_deg}, // dockPos

      {0_m, 0_m, 0_deg}, // stealPos
      {0_m, 0_m, 0_deg}, // taxiPos
      {0_m, 0_m, 0_deg},  // subStationWaitPos

    }, /*  Middle Poses - END  */
    {  /*  Bottom Poses - START  */
      {72.061_in, 20.208_in, 0_deg}, // startPos
      {82.138_in, 76.783_in, 0_deg}, // dock_LineUp_Pos
      {124.158_in, 75.594_in, 0_deg}, // dockPos
      {288.932_in, 35.832_in, 0_deg}, // stealPos
      {221.744_in, 17.986_in, 0_deg}, // taxiPos

      {0_m, 0_m, 0_deg},  // subStationWaitPos
      {  /*  Collect Path - START  */
        {72.061_in, 20.208_in, 0_deg}, // startingFromPos
        {278.389_in, 36.185_in, 0_deg}, // retrievePiece1Pos
        {132.81_in, 60.185_in, 0_deg}, // returnPiece1Pos
        {278.389_in, 84.185_in, 0_deg}, // retrievePiece2Pos
        {132.81_in, 108.185_in, 0_deg}, // returnPiece2Pos
        {278.389_in, 132.185_in, 0_deg}, // retrievePiece3Pos
        {132.81_in, 156.185_in, 0_deg}, // returnPiece3Pos
        {278.389_in, 180.185_in, 0_deg}, // retrievePiece4Pos
        {132.81_in, 180.185_in, 0_deg} // returnPiece4Pos
      }  /*  Collect Path - END  */
    }, /*  Bottom Poses - END  */
    {  /*  Game Piece Poses - START  */
      {278.389_in, 36.185_in, 0_deg},  // the bottom-most game piece
      {278.389_in, 84.185_in, 0_deg},  // the bottom-middle game piece
      {278.389_in, 132.185_in, 0_deg},  // the top-middle game piece
      {278.389_in, 180.185_in, 0_deg}  // the top-most game piece
    }  /* Game Piece Poses - END  */
  };

  AlliancePoses alliancePoses;
};


struct Drivebase {
  wom::SwerveDrive *swerve;
  wom::NavX *gyro;
};
struct AutoPathDetails {
  frc::Pose2d startPos;
  std::shared_ptr<behaviour::Behaviour> endPathing;
};

enum StartingConfig {
  Top,
  Middle,
  Bottom
};
enum EndingConfig {
  Dock,
  Steal,
  Collect,
  PrepareManual, // get rdy to go collect from substation
  Taxi,
  CollectDock
};

/*
for CollectDock, could move the Collect out of the switch statement
we can treat it like an else condition, and then we do if (collect) and if (collectDock), both using same variable names
*/

AutoPathDetails GetAutoPathingDetails(Drivebase drivebase, StartingConfig startConfig, EndingConfig endConfig, bool blueAlliance, int calledFromID, std::vector<frc::Pose2d> adjustmentPoses = {});


std::shared_ptr<behaviour::Behaviour> DockBot(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Single(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Double(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Triple(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Quad(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);
