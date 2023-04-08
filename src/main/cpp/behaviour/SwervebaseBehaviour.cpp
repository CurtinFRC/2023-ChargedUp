#include "behaviour/SwerveBaseBehaviour.h"

#include <units/angular_velocity.h>
#include <units/charge.h>
#include <units/moment_of_inertia.h>

#include "ControlUtil.h"

#include "XInputController.h"
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>

using namespace wom;

// Code for Manual Drivebase
ManualDrivebase::ManualDrivebase(wom::SwerveDrive *swerveDrivebase, frc::XboxController *driverController) : _swerveDrivebase(swerveDrivebase), _driverController(driverController) {
  Controls(swerveDrivebase);
}

void ManualDrivebase::OnStart(units::second_t dt) {
  _swerveDrivebase->OnStart();
  _swerveDrivebase->SetAccelerationLimit(6_mps_sq);
  std::cout << "Manual Drivebase Start" << std::endl;
}

void ManualDrivebase::OnTick(units::second_t deltaTime) {

  if (_driverController->GetBButton()) {
    std::cout << "RESETING POSE" << std::endl;
    _swerveDrivebase->ResetPose(frc::Pose2d());
  }

  if (_driverController->GetLeftBumperPressed()){
    maxMovementMagnitude = highSensitivityDriveSpeed;
    maxRotationMagnitude = highSensitivityRotateSpeed;
  }
  if (_driverController->GetRightBumperPressed()){
    maxMovementMagnitude = lowSensitivityDriveSpeed;
    maxRotationMagnitude = lowSensitivityRotateSpeed;
  }

  //TODO remove
  if (_driverController->GetAButtonReleased()) {
    if (isZero) {
      isZero = false;
    } else {
      isZero = true;
    }
  } 

  if (isZero) {
    _swerveDrivebase->SetZeroing();
  }
  else {
    double xVelocity = wom::spow2(-wom::deadzone(_driverController->GetLeftY(), driverDeadzone));  // GetLeftY due to x being where y should be on field
    double yVelocity = wom::spow2(-wom::deadzone(_driverController->GetLeftX(), driverDeadzone));

    double r_x = wom::spow2(-wom::deadzone(_driverController->GetRightX(), turningDeadzone));
    double r_y = wom::spow2(-wom::deadzone(_driverController->GetRightY(), turningDeadzone));


    double turnX = _driverController->GetRightX();   double turnY = _driverController->GetRightY();
    double num = sqrt(turnX * turnX + turnY * turnY);
    if (num < turningDeadzone) {
      turnX = 0;  turnY = 0;
    }

    if (_swerveDrivebase->GetIsFieldRelative()) {  // Field Relative Controls
      _swerveDrivebase->SetFieldRelativeVelocity(wom::FieldRelativeSpeeds{
        xVelocity * maxMovementMagnitude,
        yVelocity * maxMovementMagnitude,
        r_x * maxRotationMagnitude
      });
    }
    else {  // Robot Relative Controls
      _swerveDrivebase->SetFieldRelativeVelocity(wom::FieldRelativeSpeeds{
        xVelocity * maxMovementMagnitude,
        yVelocity * maxMovementMagnitude,
        r_x * maxRotationMagnitude
      });
    }
  }
} 

// Code for Drivebase Pose Controls
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

// Code for Drivebase balancing on the chargestation
DrivebaseBalance::DrivebaseBalance(wom::SwerveDrive *swerveDrivebase, wom::NavX *gyro) : _swerveDrivebase(swerveDrivebase), _gyro(gyro) {
  Controls(swerveDrivebase);
}
void DrivebaseBalance::OnTick(units::second_t deltaTime) {
  units::meters_per_second_t lateralMotorSpeed = lateralBalancePID.Calculate(_gyro->GetPitch(), deltaTime);
  units::meters_per_second_t sidewaysMotorSpeed = sidwaysBalancePID.Calculate(-_gyro->GetRoll(), deltaTime);
  _swerveDrivebase->SetVelocity(frc::ChassisSpeeds{
    // units::math::min(units::math::max(lateralMotorSpeed, -0.8), 0.8),
    -lateralMotorSpeed,
    -sidewaysMotorSpeed,
    0_deg / 1_s
  });

  _swerveDriveTable->GetEntry("Pitch").SetDouble(_gyro->GetPitch().convert<units::degree>().value());
  _swerveDriveTable->GetEntry("BalanceLateralSpeed").SetDouble(lateralMotorSpeed.value());
  _swerveDriveTable->GetEntry("BalanceSidewaysSpeed").SetDouble(sidewaysMotorSpeed.value());
}

// Code for x-ing the wheels on the drivebase
XDrivebase::XDrivebase(wom::SwerveDrive *swerveDrivebase) : _swerveDrivebase(swerveDrivebase) {   Controls(swerveDrivebase);   }
void XDrivebase::OnTick(units::second_t deltaTime) {   _swerveDrivebase->SetXWheelState();   }

// Code for auto aligning to the nearest grid position
AlignDrivebaseToNearestGrid::AlignDrivebaseToNearestGrid(wom::SwerveDrive *swerveDrivebase, std::vector<frc::Pose2d*> gridPoses) : _swerveDrivebase(swerveDrivebase), _gridPoses(gridPoses) {   Controls(swerveDrivebase);   }

void AlignDrivebaseToNearestGrid::OnStart(){
  units::degree_t alignAngle = 0_deg;
  frc::Pose2d currentPose = _swerveDrivebase->GetPose();
  double angle = std::fmod(currentPose.Rotation().Degrees().value(), 360);
  if (90 < angle && angle <= 270){   alignAngle = 180_deg;   }

  frc::Pose2d *nearestGrid = _gridPoses[0];


  for (frc::Pose2d *pose : _gridPoses) {
    frc::Pose2d difference = currentPose.RelativeTo(*pose);
    double distance = pow(difference.X().value(), 2) + pow(difference.Y().value(), 2);
    if (distance < pow(nearestGrid->X().value(), 2) + pow(nearestGrid->Y().value(), 2)){
      nearestGrid = pose;
    }
  }
  if (pow(nearestGrid->X().value(), 2) + pow(nearestGrid->Y().value(), 2) < 2){
    _swerveDrivebase->SetPose(frc::Pose2d{nearestGrid->X(), nearestGrid->Y(), alignAngle});
  }
}

void AlignDrivebaseToNearestGrid::OnTick(units::second_t deltaTime) { }