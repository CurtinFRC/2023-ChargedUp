// #include "Auto.h"
// #include "Poses.h"

// #include "behaviour/SwerveBaseBehaviour.h"
// #include "behaviour/ArmavatorBehaviour.h"

// using namespace behaviour;

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