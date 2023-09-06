// #pragma once

// // Local includes
// #include "LimeLightVision.h"

// // wom includes
// #include "behaviour/Behaviour.h"
// #include "drivetrain/SwerveDrive.h"

// // std lib includes
// #include <vector>
// #include <memory>


// // units includes

// // wpilib includes
// #include <frc/geometry/Pose3d.h>
// #include <frc/geometry/Pose2d.h>
// #include "frc/XboxController.h"

// // other external includes

// enum class poseState {
//   atPose,
//   notAtPose
// };

// class AlignBehaviour : public behaviour::Behaviour {
//  public:
//   AlignBehaviour(Limelight *limelight, frc::XboxController &driver, wom::SwerveDrive *swerve, frc::Pose3d _pose);

//   void OnTick(units::second_t dt) override;

//   void AlignToTarget(frc::Pose3d pose);
//   void CancelAlign();

// private:
//   Limelight *_limelight;
//   frc::XboxController &_driver;
//   wom::SwerveDrive *_swerve;

//   std::vector<frc::Pose3d> _SetPoints;
//   frc::Pose2d _pose;


//   poseState _state = poseState::notAtPose;
// };
