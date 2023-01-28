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
  double l_y = wom::spow2(wom::deadzone(_driverController->GetLeftX(), driverDeadzone));
  double r_x = wom::spow2(wom::deadzone(_driverController->GetRightX(), turningDeadzone));

  // _swerveDrivebase->GetConfig();

  
  
  // Robot Relative Controls
  // _swerveDrivebase->SetVelocity(frc::ChassisSpeeds {
  //   l_x * maxMovementMagnitude,
  //   l_y * maxMovementMagnitude,
  //   r_x * 360_deg / 0.01_s // were once 90_deg / 1_s
  // });

  // Field Relative Controls
  _swerveDrivebase->SetFieldRelativeVelocity(wom::FieldRelativeSpeeds {
   l_x * l_x * maxMovementMagnitude,
   l_y * l_y * maxMovementMagnitude,
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
  // determine if it's moving, speed based off of roll back speed

  // get a feel for the wheel before doing this
}
