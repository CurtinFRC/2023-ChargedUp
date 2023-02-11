#pragma once

#include <frc/geometry/Pose2d.h>

struct Poses {
    static constexpr frc::Pose2d innerGrid1{0.1_m, 0.1_m, 0_deg}; // Closest grid position to the Wall
    static constexpr frc::Pose2d innerGrid2{0.5_m, 0_m, 0_deg}; // Middle of Inner Grid
    static constexpr frc::Pose2d innerGrid3 ={0.5_m, -0.5_m, 0_deg}; // Centremost Inner Grid position
    static constexpr frc::Pose2d centreGrid1 ={0_m, 0.5_m, 0_deg}; // The non central grid on the Inner Grid side
    static constexpr frc::Pose2d centreGrid2 ={0_m, 0_m, 270_deg}; // The middle most grid 
    static constexpr frc::Pose2d centreGrid3 ={0_m, -0.5_m, 0_deg}; // The non central grid on the Outer Grid side
    static constexpr frc::Pose2d outerGrid1 ={-0.5_m, 0.5_m, 0_deg}; // Centremost outer grid position
    static constexpr frc::Pose2d outerGrid2 ={1_m, 0_m, 0_deg}; // Middle of Outer Grid
    static constexpr frc::Pose2d outerGrid3 ={-0.5_m, -0.5_m, 0_deg}; // Closest grid position to enemy Loading Zone
};