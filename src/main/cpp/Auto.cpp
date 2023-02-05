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