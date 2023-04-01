// #include "behaviour/VisionBehaviour.h"
// #include "behaviour/SwerveBaseBehaviour.h"

// VisionBehaviour::VisionBehaviour(Vision *vision, wom::SwerveDrive *swerveDrivebase, frc::XboxController *driver) : _vision(vision), _swerveDrivebase(swerveDrivebase), _driver(driver), behaviour::Behaviour("test")
//   {
//     Controls(_vision);
//   }

// void VisionBehaviour::OnTick(units::second_t dt) {
  
// }


// bool VisionBehaviour::IsAtSetPoseVision(VisionConfig _config, frc::Pose2d targetPosition)  {
//   auto currentPose = _vision->estimatePose(_config);
//   frc::Pose2d currentPose2d{currentPose.X(),currentPose.Y(), currentPose.Rotation().ToRotation2d()};
//   if (currentPose2d == targetPosition) {return true;} else {return false;}
// }


// // Vision vision;

// // void AlignmentBehaviour::CheckAlign(frc::Pose2d targetPosition, VisionConfig config, wom::SwerveDrive *swerveDrive) {
// //   // do swerve thing

// //   // check position
// // auto currentPose = vision.estimatePose(config);
// // frc::Pose2d currentPose2d{currentPose.X(),currentPose.Y(), currentPose.Rotation().ToRotation2d()};
// //   // check
// //   if (currentPose2d == targetPosition) {/*hooray*/} 
// //   else {/*repeat*/};
// // };