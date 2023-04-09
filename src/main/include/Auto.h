#pragma once

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"
#include "Armavator.h"
#include "Gripper.h"
#include "Poses.h"


struct Drivebase {
  wom::SwerveDrive *swerve;
  wom::NavX *gyro;
};
struct RoboPackage {
  Drivebase swervePackage;
  Armavator *armavator;
  Gripper *gripper;
};

/*
for CollectDock, could move the Collect out of the switch statement
we can treat it like an else condition, and then we do if (collect) and if (collectDock), both using same variable names
*/

// AutoPathDetails GetAutoPathingDetails(Drivebase drivebase, StartingConfig startConfig, EndingConfig endConfig, bool blueAlliance, int calledFromID, std::vector<frc::Pose2d> adjustmentPoses = {});

// std::shared_ptr<behaviour::Behaviour> DockBot(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

// std::shared_ptr<behaviour::Behaviour> Single(Drivebase drivebase, Armavator *armavator , Gripper *gripper, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

// std::shared_ptr<behaviour::Behaviour> Double(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

// std::shared_ptr<behaviour::Behaviour> Triple(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

// std::shared_ptr<behaviour::Behaviour> Quad(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

// std::shared_ptr<behaviour::Behaviour> Quintuple(Drivebase drivebase, bool blueAlliance, StartingConfig startConfig, EndingConfig endConfig);

std::shared_ptr<behaviour::Behaviour> ForwardDrive(RoboPackage robotPackage);

std::shared_ptr<behaviour::Behaviour> Balance(RoboPackage robotPackage);


std::shared_ptr<behaviour::Behaviour> LowPlace(RoboPackage robotPackage);
std::shared_ptr<behaviour::Behaviour> HighPlace(RoboPackage robotPackage);
std::shared_ptr<behaviour::Behaviour> Taxi(RoboPackage robotPackage);
std::shared_ptr<behaviour::Behaviour> Dock(RoboPackage robotPackage);

std::shared_ptr<behaviour::Behaviour> Single(RoboPackage robotPackage);