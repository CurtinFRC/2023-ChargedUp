#pragma once

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"

std::shared_ptr<behaviour::Behaviour> Drive(wom::SwerveDrive *swerve, wom::NavX *gyro);


class DefinedPoses {
 public:
  struct Poses {
    frc::Pose2d startPos;
    frc::Pose2d dock_LineUp_Pos;  frc::Pose2d dockPos;
    frc::Pose2d stealPos;  frc::Pose2d taxiPos;
    frc::Pose2d subStationWaitPos;
  };
  Poses poseSet{};

  Poses top_Blue {
    {0_m, 0_m, 0_deg}, // startPos
    {0_m, -1.5_m, 0_deg}, // dock_LineUp_Pos
    {2_m, -1.5_m, 0_deg}, // dockPos
    {0_m, 0_m, 0_deg}, // stealPos
    {0_m, 0_m, 0_deg}, // taxiPos
    {0_m, 0_m, 0_deg}  // subStationWaitPOs
  };
  Poses middle_Blue {
    {0_m, 0_m, 0_deg}, // startPos
    {0_m, 0_m, 0_deg}, // dock_LineUp_Pos
    {0_m, 0_m, 0_deg}, // dockPos
    {0_m, 0_m, 0_deg}, // stealPos
    {0_m, 0_m, 0_deg}, // taxiPos
    {0_m, 0_m, 0_deg}  // subStationWaitPOs
  };
  Poses bottom_Blue {
    {0_m, 0_m, 0_deg}, // startPos
    {0_m, 0_m, 0_deg}, // dock_LineUp_Pos
    {0_m, 0_m, 0_deg}, // dockPos
    {0_m, 0_m, 0_deg}, // stealPos
    {0_m, 0_m, 0_deg}, // taxiPos
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


struct SwervePack { // contains the gyro as well
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





AutoPathDetails GetAutoPathingDetails(SwervePack swerve, StartingConfig startConfig, EndingConfig endConfig, bool blueAlliance, std::vector<frc::Pose2d> adjustmentPoses = {});


std::shared_ptr<behaviour::Behaviour> DockBot(SwervePack swerve, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Single(SwervePack swerve, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Double(SwervePack swerve, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Triple(SwervePack swerve, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> Quad(SwervePack swerve, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);
