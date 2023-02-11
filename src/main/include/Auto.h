#pragma once

#include "behaviour/Behaviour.h"
#include "drivetrain/SwerveDrive.h"



/*
Starting Poses corresponding with:
    Top:  frc::Pose2d{0_m, 0_m, 0_deg}
    Middle: frc::Pose2d{0_m, 0_m, 0_deg}
    Bottom: frc::Pose2d{0_m, 0_m, 0_deg}
*/

std::shared_ptr<behaviour::Behaviour> Drive(wom::SwerveDrive *swerve, wom::NavX *gyro);


enum endingConfig {
    Dock,
    Steal,
    Collect,
    PrepareManual, // get rdy to go collect from substation
    Taxi
};

enum startingConfig {
    Top,
    Middle,
    Bottom
};




// std::shared_ptr<behaviour::Behaviour> Dock(wom::SwerveDrive *swerve, bool blueAlliance, enum startingConfig, enum endingConfig);

// std::shared_ptr<behaviour::Behaviour> Single(wom::SwerveDrive *swerve, bool blueAlliance, bool dock, enum startPos, enum endPos);

// std::shared_ptr<behaviour::Behaviour> Double(wom::SwerveDrive *swerve, bool blueAlliance, bool dock, enum startPos, enum endPos);

// std::shared_ptr<behaviour::Behaviour> Triple(wom::SwerveDrive *swerve, bool blueAlliance, bool dock, enum startPos, enum endPos);

// std::shared_ptr<behaviour::Behaviour> Quad(wom::SwerveDrive *swerve, bool blueAlliance, bool dock, enum startPos, enum endPos);













// Assuming in auto we only score high

// BLUE

// Docking Only
std::shared_ptr<behaviour::Behaviour> BLUE_Top_Dock(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> BLUE_Middle_Dock(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> BLUE_Bottom_Dock(wom::SwerveDrive *swerve);

// Single Score                 <- We should not need to move for this
std::shared_ptr<behaviour::Behaviour> BLUE_Single(wom::SwerveDrive *swerve);

// Single Score + Dock          <- We should only be in middle for doing this one
std::shared_ptr<behaviour::Behaviour> BLUE_Single_Dock(wom::SwerveDrive *swerve);

// Triple Score                 <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> BLUE_Top_Triple(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> BLUE_Bottom_Triple(wom::SwerveDrive *swerve);

// Double Score + Dock          <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> BLUE_Top_Double_Dock(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> BLUE_Bottom_Double_Dock(wom::SwerveDrive *swerve);

// Double                       <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> BLUE_Top_Double(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> BLUE_Bottom_Double(wom::SwerveDrive *swerve);

// Quad Collect                 <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> BLUE_Top_Quad_Collect(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> BLUE_Bottom_Quad_Collect(wom::SwerveDrive *swerve);



// RED

// Docking Only
std::shared_ptr<behaviour::Behaviour> RED_Top_Dock(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> RED_Middle_Dock(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> RED_Bottom_Dock(wom::SwerveDrive *swerve);

// Single Score                 <- We should not need to move for this
std::shared_ptr<behaviour::Behaviour> RED_Single(wom::SwerveDrive *swerve);

// Single Score + Dock          <- We should only be in middle for doing this one
std::shared_ptr<behaviour::Behaviour> RED_Single_Dock(wom::SwerveDrive *swerve);

// Triple Score                 <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> RED_Top_Triple(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> RED_Bottom_Triple(wom::SwerveDrive *swerve);

// Double Score + Dock          <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> RED_Top_Double_Dock(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> RED_Bottom_Double_Dock(wom::SwerveDrive *swerve);

// Double                       <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> RED_Top_Double(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> RED_Bottom_Double(wom::SwerveDrive *swerve);

// Quad Collect                 <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> RED_Top_Quad_Collect(wom::SwerveDrive *swerve);
std::shared_ptr<behaviour::Behaviour> RED_Bottom_Quad_Collect(wom::SwerveDrive *swerve);

