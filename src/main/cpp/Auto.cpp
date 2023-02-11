#include "Auto.h"
#include "Poses.h"

#include "behaviour/SwerveBaseBehaviour.h"
#include "behaviour/ArmavatorBehaviour.h"


using namespace behaviour;

// std::shared_ptr<behaviour::Behaviour> BlueSinglePiece() {
//   return (
//         make<DrivebasePoseBehaviour>(drivetrain, Poses::innerGrid1)
//         & make<ArmavatorGoToPositionBehaviour>(armavator, { 1_m, 180_deg })
//       )
//       <<  make<GripperReleaseBehaviour>(intake)
//       <<  make<ArmavatorGoToPositionBehaviour>(armavator, {1.3_m, 45_deg})
//       <<  make<ArmavatorGoToPositionBehaviour>(armavator, {1.0_m, -80_deg})
//       <<  make<DrivebasePoseBehaviour>(drivetrain, ...);

//   // each action, add a << before, similar to std::cout

// }


std::shared_ptr<behaviour::Behaviour> CircularPathing(wom::SwerveDrive *swerve) {
    return 
        make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{1_m, 0_m, 0_deg})
        << make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{2_m, 1_m, 90_deg});
        // make<DrivebasePoseBehaviour>(swerve, Poses::innerGrid1)
        // << make<WaitTime>(1_s)
        // << make<DrivebasePoseBehaviour>(swerve, Poses::innerGrid2)
        // << make<WaitTime>(1_s)
        // << make<DrivebasePoseBehaviour>(swerve, Poses::centreGrid2)
        // << make<WaitTime>(1_s)
        // << make<DrivebasePoseBehaviour>(swerve, Poses::outerGrid2)
        // << make<WaitTime>(1_s)
        // << make<DrivebasePoseBehaviour>(swerve, Poses::outerGrid1);
}


std::shared_ptr<behaviour::Behaviour> Drive(wom::SwerveDrive *swerve, wom::NavX *gyro){
    auto wait_until2 = make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{0_in, 1.5_m, 0_deg}) | make<WaitTime>(2_s); 
    return
    make<WaitTime>(1_s)
    // << make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{0_in, -1.8_m, 0_deg})
    // << wait_until2
    << make<DrivebasePoseBehaviour>(swerve, frc::Pose2d{2_m, 0_m, 0_deg})
    << make<DrivebaseBalance>(swerve, gyro);
}






// std::shared_ptr<behaviour::Behaviour> Dock(wom::SwerveDrive *swerve, bool blueAlliance, enum startPos, enum endPos){

// }

// std::shared_ptr<behaviour::Behaviour> Single(wom::SwerveDrive *swerve, bool blueAlliance, bool dock, enum startPos, enum endPos){

// }

// std::shared_ptr<behaviour::Behaviour> Double(wom::SwerveDrive *swerve, bool blueAlliance, bool dock, enum startPos, enum endPos){

// }

// std::shared_ptr<behaviour::Behaviour> Triple(wom::SwerveDrive *swerve, bool blueAlliance, bool dock, enum startPos, enum endPos){

// }

// std::shared_ptr<behaviour::Behaviour> Quad(wom::SwerveDrive *swerve, bool blueAlliance, bool dock, enum startPos, enum endPos){

// }










// Assuming in auto we only score high

// BLUE

// Docking Only
std::shared_ptr<behaviour::Behaviour> BLUE_Top_Dock(wom::SwerveDrive *swerve){
    // assumes Top is in placement position for outergrid_1
    return
    /*
    relative drive frc::Pose2d{0.3_m, -62.39_in, 0_deg}
    relative drive frc::Pose2d{68.3_in, 0_m, 0_deg}
    activate swerve balance behaviour
    */
    make<WaitTime>(1_s);
}
std::shared_ptr<behaviour::Behaviour> BLUE_Middle_Dock(wom::SwerveDrive *swerve){
    // assumes Middle is in placement position for centregrid_2
    return
    /*
    relative drive frc::Pose2d{68.3_in, 0_m, 0_deg}
    activate swerve balance behaviour
    */
    make<WaitTime>(1_s);
}
std::shared_ptr<behaviour::Behaviour> BLUE_Bottom_Dock(wom::SwerveDrive *swerve){
    // assumes Bottom is in placement position for innergrid_1
    return
    /*
    relative drive frc::Pose2d{0.3_m, 62.39_in, 0_deg}
    relative drive frc::Pose2d{68.3_in, 0_m, 0_deg}
    activate swerve balance behaviour
    */
    make<WaitTime>(1_s);
}

// Single Score                 <- We should not need to move for this
std::shared_ptr<behaviour::Behaviour> BLUE_Single(wom::SwerveDrive *swerve){
    return
    /*
    make<ArmavatorScoreHigh>();
    */
    make<WaitTime>(1_s);
}

// Single Score + Dock          <- We should only be in middle for doing this one
std::shared_ptr<behaviour::Behaviour> BLUE_Single_Dock(wom::SwerveDrive *swerve){
    return
    /*
    make<ArmavatorScoreHigh>();
    make<ArmavatorGoToBalancePosition() & relative drive frc::Pose2d{68.3_in, 0_m, 0_deg}
    activate swerve balance behaviour
    */
    make<WaitTime>(1_s);
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
std::shared_ptr<behaviour::Behaviour> BLUE_Bottom_Triple(wom::SwerveDrive *swerve){
    return make<WaitTime>(1_s);
}

// Double Score + Dock          <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> BLUE_Top_Double_Dock(wom::SwerveDrive *swerve){
    return make<WaitTime>(1_s);
}
std::shared_ptr<behaviour::Behaviour> BLUE_Bottom_Double_Dock(wom::SwerveDrive *swerve){
    return make<WaitTime>(1_s);
}

// Double                       <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> BLUE_Top_Double(wom::SwerveDrive *swerve){
    return make<WaitTime>(1_s);
}
std::shared_ptr<behaviour::Behaviour> BLUE_Bottom_Double(wom::SwerveDrive *swerve){
    return make<WaitTime>(1_s);
}

// Quad Collect                 <- We should never be in the middle for doing this one
std::shared_ptr<behaviour::Behaviour> BLUE_Top_Quad_Collect(wom::SwerveDrive *swerve){
    return make<WaitTime>(1_s);
}
std::shared_ptr<behaviour::Behaviour> BLUE_Bottom_Quad_Collect(wom::SwerveDrive *swerve){
    return make<WaitTime>(1_s);
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