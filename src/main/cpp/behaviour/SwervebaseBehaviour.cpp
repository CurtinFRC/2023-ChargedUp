#include "behaviour/SwerveBaseBehaviour.h"

#include <units/angular_velocity.h>
#include <units/charge.h>
#include <units/moment_of_inertia.h>

#include "ControlUtil.h"

// #include <units/units.h>

using namespace wom;

// Code for Manual Drivebase

ManualDrivebase::ManualDrivebase(wom::SwerveDrive *swerveDrivebase, frc::PS4Controller *driverController) : _swerveDrivebase(swerveDrivebase), _driverController(driverController) {
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
  double r_y = wom::spow2(-wom::deadzone(_driverController->GetRightX(), turningDeadzone));
  
  if (_swerveDrivebase->GetIsFieldRelative()) {  // Field Relative Controls
    _swerveDrivebase->SetFieldRelativeVelocity(wom::FieldRelativeSpeeds{
        xVelocity * maxMovementMagnitude,
        yVelocity * maxMovementMagnitude,
        r_x * 360_deg / 1_s
    }); 
  } else {  // Robot Relative Controls
    _swerveDrivebase->SetVelocity(frc::ChassisSpeeds{
        xVelocity * maxMovementMagnitude,
        yVelocity * maxMovementMagnitude,
        r_x * 360_deg / 1_s
    });
  }

  frc::Pose2d currentPose = _swerveDrivebase->GetPose();
  units::degree_t currentAngle = currentPose.Rotation().Degrees();
  units::degree_t requestedAngle = currentAngle;

  if (r_x > 0 && r_y > 0) { // Quadrant 1
    requestedAngle = (1_rad * atan2(r_y, r_x));
  } else if (r_x < 0 && r_y > 0) { // Quadrant 2
    requestedAngle = 180_deg - (1_rad * atan2(r_y, r_x));
  } else if (r_x < 0 && r_y < 0) { // Quadrant 3
    requestedAngle = 180_deg + (1_rad * atan2(r_y, r_x));
  } else if (r_x > 0 && r_y < 0) { // Quadrant 4
    requestedAngle = 360_deg - (1_rad * atan2(r_y, r_x));
  }
  if (r_x == 0) {
    if (r_y > 0){   requestedAngle = 90_deg;   }
    else if (r_y < 0){   requestedAngle = 270_deg;   }
  }
  if (r_y == 0){
    if (r_x > 0){   requestedAngle = 0_deg;   }
    else if (r_x < 0){   requestedAngle = 180_deg;   }
  }


  units::meter_t newX = currentPose.X() - xVelocity * maxMovementMagnitude * deltaTime;
  units::meter_t newY = currentPose.Y() - yVelocity * maxMovementMagnitude * deltaTime;
  _swerveDrivebase.SetPose(frc::Pose2d(newX, newY, requestedAngle));




  _swerveDriveTable->GetEntry("isFieldOrientated").SetBoolean(isFieldOrientated); // allows for a user to know if the robot is in field relative, or robot relative mode
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
    lateralMotorSpeed,
    sidewaysMotorSpeed,
    0_deg / 1_s
  });

  _swerveDriveTable->GetEntry("Pitch").SetDouble(_gyro->GetPitch().convert<units::degree>().value());
  _swerveDriveTable->GetEntry("BalanceLateralSpeed").SetDouble(lateralMotorSpeed.value());
  _swerveDriveTable->GetEntry("BalanceSidewaysSpeed").SetDouble(sidewaysMotorSpeed.value());
}



// Code for x-ing the wheels on the drivebase

XDrivebase::XDrivebase(wom::SwerveDrive *swerveDrivebase) : _swerveDrivebase(swerveDrivebase) {   Controls(swerveDrivebase);   }
void XDrivebase::OnTick(units::second_t deltaTime) {   _swerveDrivebase->SetXWheelState();   }




AlignDrivebaseToNearestGrid::AlignDrivebaseToNearestGrid(wom::SwerveDrive *swerveDrivebase, std::vector<frc::Pose2d*> gridPoses) : _swerveDrivebase(swerveDrivebase), _gridPoses(gridPoses) {   Controls(swerveDrivebase);   }

void AlignDrivebaseToNearestGrid::OnStart(){
  frc::Pose2d currentPose = _swerveDrivebase->GetPose();
  units::degree_t alignAngle = 0_deg;
  if (90 < std::fmod(currentPose.Rotation().Degrees().value(), 360) <= 270){   alignAngle = 180_deg;   }

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

void AlignDrivebaseToNearestGrid::OnTick(units::second_t deltaTime){

}