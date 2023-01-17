#include "behaviour/SwerveBaseBehaviour.h"
#include "ControlUtil.h"

using namespace wom;

ManualDrivebase::ManualDrivebase(wom::SwerveDrive *swerveDrivebase, frc::XboxController *driverController):  _swerveDrivebase(swerveDrivebase), _driverController(driverController){
    Controls(swerveDrivebase);
}

void ManualDrivebase::OnTick(units::second_t deltaTime){
  double l_x = wom::deadzone(_driverController->GetLeftX(), driverDeadzone);
  double l_y = wom::deadzone(_driverController->GetLeftY(), driverDeadzone);
  double r_x = wom::deadzone(_driverController->GetRightX(), turningDeadzone);


  // ALL NEW STUFF

  // currently using an Xbox controller for testing, but a button bar like thing would be ideal
  // 9 grid positions
  // D-pad selects for inner 3 and 1 inner coop grid
  // D-pad + A selects for outer 3 and 1 inner coop grid
  // A + X selects the inner coop grid

  // Up (0)  Right (90)  Down (180)  Left (270)

  // struct PoseButtonRelationship{
  //   frc::Pose2d innerGrid1 = frc::Pose2d(1_m, 0_m, 0_rad); // -> coopGrid3
  //   frc::Pose2d innerGrid2 = frc::Pose2d(1_m, 1_m, 2_rad); // -> outerGrid1
  //   frc::Pose2d innerGrid3 = frc::Pose2d(1_m, 2_m, 4_rad); // -> outerGrid2
  //   frc::Pose2d coopGrid1 = frc::Pose2d(2_m, 0_m, 0_rad); // -> outerGrid3
  //   // I can just add a constant number to these, hence I can avoid the last 4
  //   frc::Pose2d coopGrid2 = frc::Pose2d(2_m, 1_m, 2_rad); // the one in the centre
  //   //frc::Pose2d coopGrid3 = frc::Pose2d(2_m, 2_m, 4_rad);
  //   //frc::Pose2d outerGrid1 = frc::Pose2d(3_m, 0_m, 0_rad);
  //   //frc::Pose2d outerGrid2 = frc::Pose2d(3_m, 1_m, 2_rad);
  //   //frc::Pose2d outerGrid3 = frc::Pose2d(3_m, 2_m, 4_rad);
  // };
  // PoseButtonRelationship poseButtons;


  // /* 
  //   COULD DO:
  //     If holding down the pose button, then issues there with unnecessary calculations
  //       can check if have moved since last pose 
  // */

  // bool isAHeld = _driverController->GetAButton();


  // const units::meter_t outerGridPortionYOffset = 4_m;

  // bool setNewPose = true;
  // frc::Pose2d newPose;

  // int dpadHeld = _driverController->GetPOV();
  // switch (dpadHeld){
  //   case -1: // dpad not held
  //     setNewPose = false;
  //     break;
  //   case 0: // dpad up
  //     newPose = poseButtons.innerGrid1;
  //     break;
  //   case 90: // dpad right
  //     newPose = poseButtons.innerGrid2;
  //     break;
  //   case 180: // dpad down
  //     newPose = poseButtons.innerGrid3;
  //     break;
  //   case 270: // dpad left
  //     newPose = poseButtons.coopGrid1;
  //     break;
  // }
  // if (_driverController->GetXButton() && isAHeld){
  //   newPose = poseButtons.coopGrid2;
  //   setNewPose = true;
  // }

  // if (setNewPose){
  //   if (isAHeld){
  //     newPose = frc::Pose2d(newPose.X(), newPose.Y() + outerGridPortionYOffset, newPose.Rotation());
  //   }
  //   _swerveDrivebase->SetPose(newPose);
  //   setNewPose = false;
  // }

  // END OF NEW STUFF
  std::cout << "Drive" << std::endl;

 

  _swerveDrivebase->SetVelocity(frc::ChassisSpeeds {
    l_x * maxMovementMagnitude,
    l_y * maxMovementMagnitude,
    r_x * 180_deg / 0.25_s
  });
}


DrivebasePoseBehaviour::DrivebasePoseBehaviour(wom::SwerveDrive *swerveDrivebase, frc::Pose2d pose)
  : _swerveDrivebase(swerveDrivebase), _pose(pose)
{
  Controls(swerveDrivebase);
}

void DrivebasePoseBehaviour::OnTick(units::second_t deltaTime){
  _swerveDrivebase->SetPose(_pose);

  if (_swerveDrivebase->IsAtSetPose()){
    SetDone();
  }
}