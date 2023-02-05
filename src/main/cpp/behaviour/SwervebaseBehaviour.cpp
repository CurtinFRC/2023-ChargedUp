#include "behaviour/SwerveBaseBehaviour.h"

#include <units/angular_velocity.h>
#include <units/charge.h>
#include <units/moment_of_inertia.h>

#include "ControlUtil.h"
// #include <units/units.h>

using namespace wom;

ManualDrivebase::ManualDrivebase(wom::SwerveDrive *swerveDrivebase, frc::XboxController *driverController) : _swerveDrivebase(swerveDrivebase), _driverController(driverController) {
  Controls(swerveDrivebase);
}

void ManualDrivebase::OnStart() {
  _swerveDrivebase->SetAccelerationLimit(6_mps_sq);
  std::cout << "Manual Drivebase Start" << std::endl;
}

void ManualDrivebase::OnTick(units::second_t deltaTime) {
  double l_x = wom::spow2(-wom::deadzone(_driverController->GetLeftY(), driverDeadzone));  // GetLeftY due to x being where y should be on field
  double l_y = wom::spow2(-wom::deadzone(_driverController->GetLeftX(), driverDeadzone));
  double r_x = wom::spow2(-wom::deadzone(_driverController->GetRightX(), turningDeadzone));


  if (_driverController->GetYButtonPressed()) {  isFieldOrientated = !isFieldOrientated;  }

  if (isFieldOrientated) {  // Field Relative Controls
    _swerveDrivebase->SetFieldRelativeVelocity(wom::FieldRelativeSpeeds{
        l_x * maxMovementMagnitude,
        l_y * maxMovementMagnitude,
        r_x * 360_deg / 1_s
    });
  } else {  // Robot Relative Controls
    _swerveDrivebase->SetVelocity(frc::ChassisSpeeds{
        l_x * maxMovementMagnitude,
        l_y * maxMovementMagnitude,
        r_x * 360_deg / 1_s
    });
  }
  _swerveDriveTable->GetEntry("isFieldOrientated").SetBoolean(isFieldOrientated);
  }

DrivebasePoseBehaviour::DrivebasePoseBehaviour(
    wom::SwerveDrive *swerveDrivebase, frc::Pose2d pose)
    : _swerveDrivebase(swerveDrivebase), _pose(pose) {
  Controls(swerveDrivebase);
}
void DrivebasePoseBehaviour::OnTick(units::second_t deltaTime) {
  double setPoseAngle = _pose.Rotation().Degrees().value();
  double difference = fmod(_swerveDrivebase->GetPose().Rotation().Degrees().value(), 360.0);

  double currentAngle = _swerveDrivebase->GetPose().Rotation().Degrees().value();
  units::degree_t adjustedAngle = 1_deg * (currentAngle - fmod(currentAngle, 360) + _pose.Rotation().Degrees().value());

  _swerveDrivebase->SetPose(frc::Pose2d{_pose.X(), _pose.Y(), adjustedAngle});


  frc::Pose2d currentPose = _swerveDrivebase->GetPose();
  _swerveDriveTable->GetEntry("SetPose X").SetDouble(_pose.X().value());
  _swerveDriveTable->GetEntry("SetPose Y").SetDouble(_pose.Y().value());
  _swerveDriveTable->GetEntry("SetPose Theta").SetDouble(_pose.Rotation().Degrees().value());
  _swerveDriveTable->GetEntry("CurrentPose X").SetDouble(currentPose.X().value());
  _swerveDriveTable->GetEntry("CurrentPose Y").SetDouble(currentPose.Y().value());
  _swerveDriveTable->GetEntry("CurrentPose Theta").SetDouble(currentPose.Rotation().Degrees().value());

  if (_swerveDrivebase->IsAtSetPose()){
    SetDone();
  }
}

DrivebaseBalance::DrivebaseBalance(wom::SwerveDrive *swerveDrivebase) : _swerveDrivebase(swerveDrivebase) {
  Controls(swerveDrivebase);
}
void DrivebaseBalance::OnTick(units::second_t deltaTime) {
  /*
    current plan:
      Drive at a large-ish speed
      When angle changes (~5_deg), set convergence point to current position #
    check if angle change is expected for driving, <- this check can lead to
    false info Reverse at a slower speed (0.9*speed + lowestPossibleSpeed)
      Repeat
  */

  /*
    speed = (x, 0)
    if (prevAngle - currentAngle >= 5){   // prev-cur due to angle being reduced
    if balanced convergencePoint = _swerveDrivebase.GetPose() _speed.x *= -0.9
    }
    distanceFromLastPoint (aka the error) = convergencePoint -
    prevConvergencePoint

    if (distanceFromLastPoint <= 2_cm) {
      swerve.XWheels()
    }

  */

  /*
  
  currentState = positiveClimbing
  if (gyro >= 13) and (currentState == negativeClimbing):
    currentState = positiveClimbing
    speed = -0.9 * speed
  else if (gyro <= -13) and (currentState == positiveClimbing):
    currentState = negativeClimbing


  */


  /*
  
  driving on the charge station, the gyro will be at an initial angle, this is the angle that we treat as positive

  initialAngle = gyro.GetAngle()   <- runs only on start
  if (gyro.GetAngle()){

  }
  
  
  */


  // if (fmod(_swerveDrivebase->GetConfig().gyro->GetAngle(), 360) >= 13){

  // }
  // else if (fmod(_swerveDrivebase->GetConfig().gyro->GetAngle(), 360) <= -13){
    
  // }

}

XDrivebase::XDrivebase(wom::SwerveDrive *swerveDrivebase) : _swerveDrivebase(swerveDrivebase) {
  Controls(swerveDrivebase);
}
void XDrivebase::OnTick(units::second_t deltaTime) {
  _swerveDrivebase->SetXWheelState();
}