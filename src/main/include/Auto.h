#pragma once

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"


class DefinedPoses {
 public:
  struct Poses {
    frc::Pose2d startPos;
    frc::Pose2d dock_LineUp_Pos;
    frc::Pose2d dockPos;
    frc::Pose2d stealPos;
    frc::Pose2d taxiPos;
    frc::Pose2d subStationWaitPos;
  };
  Poses poseSet{};

  Poses top_Blue {
    {72.055_in, 196.3_in, 0_deg}, // startPos
    {83.607_in, 139.388_in, 0_deg}, // dock_LineUp_Pos
    {122.808_in, 139.286_in, 0_deg}, // dockPos
    {287.887_in, 180.555_in, 0_deg}, // stealPos
    {171.061_in, 195.748_in, 0_deg}, // taxiPos

    {291.192_in, 246.31_in, 0_deg}  // subStationWaitPOs
  };
  Poses middle_Blue {
    {72.043_in, 86.608_in, 0_deg}, // startPos
    {83.629_in, 107.795_in, 0_deg}, // dock_LineUp_Pos
    {122.407_in, 107.992_in, 0_deg}, // dockPos

    {0_m, 0_m, 0_deg}, // stealPos
    {0_m, 0_m, 0_deg}, // taxiPos
    {0_m, 0_m, 0_deg}  // subStationWaitPOs
  };
  Poses bottom_Blue {
    {72.061_in, 20.208_in, 0_deg}, // startPos
    {82.138_in, 76.783_in, 0_deg}, // dock_LineUp_Pos
    {124.158_in, 75.594_in, 0_deg}, // dockPos
    {288.932_in, 35.832_in, 0_deg}, // stealPos
    {221.744_in, 17.986_in, 0_deg}, // taxiPos

    {0_m, 0_m, 0_deg}  // subStationWaitPOs
  };

  Poses top_Red {
    {0_m, 0_m, 0_deg}, // startPos
    {0_m, 0_m, 0_deg}, // dock_LineUp_Pos
    {0_m, 0_m, 0_deg}, // dockPos
    {0_m, 0_m, 0_deg}, // stealPos
    {0_m, 0_m, 0_deg}, // taxiPos
    {0_m, 0_m, 0_deg}  // subStationWaitPOs
  };
  Poses middle_Red {
    {0_m, 0_m, 0_deg}, // startPos
    {0_m, 0_m, 0_deg}, // dock_LineUp_Pos
    {0_m, 0_m, 0_deg}, // dockPos
    {0_m, 0_m, 0_deg}, // stealPos
    {0_m, 0_m, 0_deg}, // taxiPos
    {0_m, 0_m, 0_deg}  // subStationWaitPOs
  };
  Poses bottom_Red {
    {0_m, 0_m, 0_deg}, // startPos
    {0_m, 0_m, 0_deg}, // dock_LineUp_Pos
    {0_m, 0_m, 0_deg}, // dockPos
    {0_m, 0_m, 0_deg}, // stealPos
    {0_m, 0_m, 0_deg}, // taxiPos
    {0_m, 0_m, 0_deg}  // subStationWaitPOs
  };
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
  Taxi
};

AutoPathDetails GetAutoPathingDetails(Drivebase drivebase, StartingConfig startConfig, EndingConfig endConfig, bool blueAlliance, int calledFromID, std::vector<frc::Pose2d> adjustmentPoses = {});


std::shared_ptr<behaviour::Behaviour> DockBot(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Single(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Double(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Triple(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Quad(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);
