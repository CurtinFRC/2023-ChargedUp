// #include "behaviour/LimelightBehaviours.h"


// AlignBehaviour::AlignBehaviour(Limelight *limelight, frc::XboxController &driver, wom::SwerveDrive *swerve, frc::Pose2d pose)
//     : _limelight(limelight), _driver(driver), _swerve(swerve), _pose(pose) {
//   Controls(limelight);
//   Controls(swerve);
// }

// void AlignBehaviour::AlignToTarget() {
//   //swerve->SetPose(pose);

//   _state = poseState::notAtPose;
// }

// void AlignBehaviour::CancelAlign() {
//   _state = poseState::atPose;
// }

// void AlignBehaviour::OnTick(units::second_t dt) {
//     if (_state == poseState::notAtPose && !_limelight->IsAtSetPoseVision(_pose, dt)) {
//         _swerve->SetPose(_pose);
//     } else {
//         _state = poseState::atPose;
//     }
// }
