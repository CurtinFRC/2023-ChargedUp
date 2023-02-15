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
  double xVelocity = wom::spow2(-wom::deadzone(_driverController->GetLeftY(), driverDeadzone));  // GetLeftY due to x being where y should be on field
  double yVelocity = wom::spow2(-wom::deadzone(_driverController->GetLeftX(), driverDeadzone));


  double r_x = wom::spow2(-wom::deadzone(_driverController->GetRightX(), turningDeadzone));
  double r_y = wom::spow2(-wom::deadzone(_driverController->GetRightY(), turningDeadzone));

  units::degree_t currentAngle = _swerveDrivebase->GetPose().Rotation().Degrees();
  units::degree_t requestedAngle = 0_deg; // = swerve.currentAngle

  if (r_x > 0 && r_y > 0){ // quad 1
    requestedAngle = 1_deg * atan2(r_y, r_x);
  } else if (r_x < 0 && r_y > 0) { // quad 2
    requestedAngle = 1_deg * (180 - atan2(r_y, r_x));
  } else if (r_x < 0 && r_y < 0){ // quad 3
    requestedAngle = 1_deg * (180 + atan2(r_y, r_x));
  } else if (r_x > 0 && r_y < 0) { // quad 4
    requestedAngle = 1_deg * (360 - atan2(r_y, r_x));
  } else if (r_x == 0) {
    if (r_y > 0) {   requestedAngle = 90_deg;   }
    else if (r_y < 0) {   requestedAngle = -90_deg;   }
  }
  // if yVelocity == 0, then atan2 = undefined


  if (_driverController->GetYButtonPressed()) {  isFieldOrientated = !isFieldOrientated;  }

  if (isFieldOrientated) {  // Field Relative Controls
    _swerveDrivebase->SetFieldRelativeVelocity(wom::FieldRelativeSpeeds{
        xVelocity * maxMovementMagnitude,
        yVelocity * maxMovementMagnitude,
        turnSpeed * 360_deg / 1_s
    }); 
  } else {  // Robot Relative Controls
    _swerveDrivebase->SetVelocity(frc::ChassisSpeeds{
        xVelocity * maxMovementMagnitude,
        yVelocity * maxMovementMagnitude,
        turnSpeed * 360_deg / 1_s
    });
  }
  _swerveDriveTable->GetEntry("isFieldOrientated").SetBoolean(isFieldOrientated);

  


  /*
  
  get current angle
  get requested angle

  // deltaTime is the time since the last call
  // the drivebase wants to turn at most 360 degrees per second
  // due to substitution, the turnSpeed will be 360_deg
  
  */

}





DrivebasePoseBehaviour::DrivebasePoseBehaviour(
    wom::SwerveDrive *swerveDrivebase, frc::Pose2d pose, bool hold)
    : _swerveDrivebase(swerveDrivebase), _pose(pose), _hold(hold) {
  Controls(swerveDrivebase);
}
void DrivebasePoseBehaviour::OnTick(units::second_t deltaTime) {
  double currentAngle = _swerveDrivebase->GetPose().Rotation().Degrees().value();
  units::degree_t adjustedAngle = 1_deg * (currentAngle - fmod(currentAngle, 360) + _pose.Rotation().Degrees().value());

  _swerveDrivebase->SetPose(frc::Pose2d{_pose.X(), _pose.Y(), adjustedAngle});

  if (_swerveDrivebase->IsAtSetPose() && !_hold){   SetDone();   }
}





DrivebaseBalance::DrivebaseBalance(wom::SwerveDrive *swerveDrivebase, wom::NavX *gyro) : _swerveDrivebase(swerveDrivebase), _gyro(gyro) {
  Controls(swerveDrivebase);
}
void DrivebaseBalance::OnTick(units::second_t deltaTime) {
  units::meters_per_second_t lateralMotorSpeed = lateralBalancePID.Calculate(_gyro->GetPitch(), deltaTime);
  units::meters_per_second_t sidewaysMotorSpeed = sidwaysBalancePID.Calculate(-_gyro->GetRoll(), deltaTime);
  _swerveDrivebase->SetVelocity(frc::ChassisSpeeds{
    // units::math::min(units::math::max(lateralMotorSpeed, -0.8), 0.8),
    lateralMotorSpeed,
    sidewaysMotorSpeed,
    0_deg / 1_s
  });

  _swerveDriveTable->GetEntry("Pitch").SetDouble(_gyro->GetPitch().convert<units::degree>().value());
  _swerveDriveTable->GetEntry("BalanceLateralSpeed").SetDouble(lateralMotorSpeed.value());
  _swerveDriveTable->GetEntry("BalanceSidewaysSpeed").SetDouble(sidewaysMotorSpeed.value());
}





XDrivebase::XDrivebase(wom::SwerveDrive *swerveDrivebase) : _swerveDrivebase(swerveDrivebase) {   Controls(swerveDrivebase);   }
void XDrivebase::OnTick(units::second_t deltaTime) {   _swerveDrivebase->SetXWheelState();   }