// #pragma once 

// #include "behaviour/Behaviour.h"
// #include"behaviour/BehaviourScheduler.h"
// #include "Vision.h"
// #include "drivetrain/SwerveDrive.h"
// #include <frc/XboxController.h>
// // #include "Robot.h"

// class AlignmentBehaviour : public behaviour::Behaviour {
//   public :
//    void OnTick(units::second_t dt);
//    void CheckAlign(frc::Pose2d targetPosition, VisionConfig config, wom::SwerveDrive *swerveDrive);
//   private :

// };

// class VisionBehaviour : public behaviour::Behaviour {
//  public:S
//   // VisionBehaviour(Vision *vision, frc::XboxController *driver);
//   VisionBehaviour(Vision *vision, wom::SwerveDrive *swerveDrivebase, frc::XboxController *driver);

//   void OnTick(units::second_t dt) override;

//   void dPadTop() {
//     if (_driver->GetXButtonPressed()) {

//     }
//   };

//   void dPadBottom() {

//   };

//   bool IsAtSetPoseVision(VisionConfig _config, frc::Pose2d targetPosition);


//  private: 
//   wom::SwerveDrive *_swerveDrivebase;
//   Vision *_vision;
//   frc::XboxController *_driver;
// };