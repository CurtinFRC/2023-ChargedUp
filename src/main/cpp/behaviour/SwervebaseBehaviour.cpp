#include "behaviour/SwerveBaseBehaviour.h"
#include "ControlUtil.h"

#include <units/angular_velocity.h>
#include <units/charge.h>
#include <units/moment_of_inertia.h>
// #include <units/units.h>

using namespace wom;

ManualDrivebase::ManualDrivebase(wom::SwerveDrive *swerveDrivebase, frc::XboxController *driverController):  _swerveDrivebase(swerveDrivebase), _driverController(driverController){
    Controls(swerveDrivebase);
}

void ManualDrivebase::OnTick(units::second_t deltaTime){
  double l_x = wom::spow2(-wom::deadzone(_driverController->GetLeftY(), driverDeadzone)); // GetLeftY due to x being where y should be
  double l_y = wom::spow2(-wom::deadzone(_driverController->GetLeftX(), driverDeadzone));
  double r_x = wom::spow2(-wom::deadzone(_driverController->GetRightX(), turningDeadzone));

  // _swerveDrivebase->GetConfig();
  // Robot Relative Controls
  // _swerveDrivebase->SetVelocity(frc::ChassisSpeeds {
  //   l_x * maxMovementMagnitude,
  //   l_y * maxMovementMagnitude,
  //   r_x * 360_deg / 0.01_s // were once 90_deg / 1_s
  // });

  // Field Relative Controls
  _swerveDrivebase->SetFieldRelativeVelocity(wom::FieldRelativeSpeeds {
   l_x * maxMovementMagnitude,
   l_y * maxMovementMagnitude,
   r_x * 360_deg / 0.01_s // were once 360_deg / 1_s
  });
  
  // Tests if the Robot Moves
  //  _swerveDrivebase->SetVelocity(frc::ChassisSpeeds {
  //    0.5_mps,
  //    0_mps,
  //    0_rad_per_s
  //  });
}

DrivebasePoseBehaviour::DrivebasePoseBehaviour(wom::SwerveDrive *swerveDrivebase, frc::Pose2d pose) : _swerveDrivebase(swerveDrivebase), _pose(pose){
  Controls(swerveDrivebase);
}
void DrivebasePoseBehaviour::OnTick(units::second_t deltaTime){
  _swerveDrivebase->SetPose(_pose);

  if (_swerveDrivebase->IsAtSetPose()){
    SetDone();
  }
}

DrivebaseBalance::DrivebaseBalance(wom::SwerveDrive *swerveDrivebase) : _swerveDrivebase(swerveDrivebase) {
  Controls(swerveDrivebase);
}
void DrivebaseBalance::OnTick(units::second_t deltaTime){
  /*
    current plan:
      Drive at a large-ish speed
      When angle changes (~5_deg), set convergence point to current position     # check if angle change is expected for driving, <- this check can lead to false info
      Reverse at a slower speed (0.9*speed + lowestPossibleSpeed)
      Repeat
  */

  /*
    speed = (x, 0)
    if (prevAngle - currentAngle >= 5){   // prev-cur due to angle being reduced if balanced
      convergencePoint = _swerveDrivebase.GetPose()
      _speed.x *= -0.9
    }
    distanceFromLastPoint (aka the error) = convergencePoint - prevConvergencePoint

    if (distanceFromLastPoint <= 2_cm) {
      swerve.XWheels()
    }

  */


}


XDrivebase::XDrivebase(wom::SwerveDrive *swerveDrivebase):  _swerveDrivebase(swerveDrivebase){
  Controls(swerveDrivebase);
}
void XDrivebase::OnTick(units::second_t deltaTime){
  _swerveDrivebase->SetXWheelState();
}