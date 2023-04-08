#include "behaviour/SwerveBaseBehaviour.h"

#include <units/angular_velocity.h>
#include <units/charge.h>
#include <units/moment_of_inertia.h>

#include "ControlUtil.h"

#include "XInputController.h"
#include <frc/XboxController.h>
#include <frc/PS4Controller.h>

#include <vector>

#include "Poses.h"

// #include <units/units.h>

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
  _swerveDrivebase->SetVoltageLimit(10_V);

  // if (_driverController->GetBButton()) {
  //   std::cout << "RESETING POSE" << std::endl;
  //   _swerveDrivebase->ResetPose(frc::Pose2d());
  // }


  //  SOLUTION TO "ANTI-TIP"
  // double joystickSpeedX = (_driverController->GetLeftX() - prevJoystickX) / deltaTime.value();
  // double JoystickSpeedY = (_driverController->GetLeftY() - prevJoystickY) / deltaTime.value();

  // if (sqrt(joystickSpeedX*joystickSpeedX + JoystickSpeedY*JoystickSpeedY) > smoothingThreshold){
  //   usingJoystickXPos = (prevPrevJoystickX + prevJoystickX + _driverController->GetLeftX()) / 3;
  //   usingJoystickYPos = (prevPrevJoystickY + prevJoystickY + _driverController->GetLeftY()) / 3;
  // } else {
    // usingJoystickXPos = _driverController->GetLeftX();
    // usingJoystickYPos = _driverController->GetLeftY();
  // }

  /*   TOGGLE SOLUTION   */
  /*if (_driverController->GetLeftBumperPressed()){
    maxMovementMagnitude = highSensitivityDriveSpeed;
    maxRotationMagnitude = highSensitivityRotateSpeed;
  }
  if (_driverController->GetRightBumperPressed()){
    maxMovementMagnitude = lowSensitivityDriveSpeed;
    maxRotationMagnitude = lowSensitivityRotateSpeed;
  }*/

  /*   HOLD SOLUTION   */
  // if (_driverController->GetLeftBumperPressed()){
  //   maxMovementMagnitude = lowSensitivityDriveSpeed;   maxRotationMagnitude = lowSensitivityRotateSpeed;
  // } else if (_driverController->GetLeftBumperReleased() & !_driverController->GetRightBumper()){
  //   maxMovementMagnitude = defaultDriveSpeed;   maxRotationMagnitude = defaultRotateSpeed;
  // }
  // if (_driverController->GetRightBumperPressed()){
  //   maxMovementMagnitude = highSensitivityDriveSpeed;   maxRotationMagnitude = highSensitivityRotateSpeed;
  // } else if (_driverController->GetRightBumperReleased() & !_driverController->GetLeftBumper()){
  //   maxMovementMagnitude = defaultDriveSpeed;   maxRotationMagnitude = defaultRotateSpeed;
  // }


  if (_driverController->GetAButtonReleased()) {
    isZero = !isZero;
  } 

  if (isZero) {
    _swerveDrivebase->SetZeroing();
  }
  else {
    double xVelocity = wom::spow2(-wom::deadzone(_driverController->GetLeftY(), driverDeadzone));  // GetLeftY due to x being where y should be on field
    double yVelocity = wom::spow2(-wom::deadzone(_driverController->GetLeftX(), driverDeadzone));
    double r_x = wom::spow2(-wom::deadzone(_driverController->GetRightX(), turningDeadzone));
    double r_y = wom::spow2(-wom::deadzone(_driverController->GetRightY(), turningDeadzone));

    // double turnX = _driverController->GetRightX();   double turnY = _driverController->GetRightY();
    // double num = sqrt(turnX * turnX + turnY * turnY);
    // if (num < turningDeadzone) {
    //   turnX = 0;   turnY = 0;
    // }

    if (_swerveDrivebase->GetIsFieldRelative()) {  // Field Relative Controls
      // units::degree_t currentAngle = _swerveDrivebase->GetPose().Rotation().Degrees();
      // CalculateRequestedAngle(turnX, turnY, currentAngle);

      // _swerveDrivebase->RotateMatchJoystick(_requestedAngle, wom::FieldRelativeSpeeds{
      //   xVelocity * maxMovementMagnitude,
      //   yVelocity * maxMovementMagnitude,
      //   r_x * maxRotationMagnitude
      // });

      _swerveDrivebase->SetFieldRelativeVelocity(wom::FieldRelativeSpeeds{
        xVelocity * maxMovementMagnitude,
        yVelocity * maxMovementMagnitude,
        r_x * maxRotationMagnitude
      });
    }
    else {  // Robot Relative Controls
      _swerveDrivebase->SetVelocity(frc::ChassisSpeeds{
          xVelocity * maxMovementMagnitude,
          yVelocity * maxMovementMagnitude,
          r_x * maxRotationMagnitude
      });
    }
  }

  // prevPrevJoystickX = prevJoystickX;
  // prevPrevJoystickY = prevJoystickY;
  // prevJoystickX = _driverController->GetLeftX();
  // prevJoystickY = _driverController->GetLeftY();
} 

// void ManualDrivebase::CalculateRequestedAngle(double joystickX, double joystickY, units::degree_t defaultAngle){
//   _requestedAngle = defaultAngle;
//   if (joystickX > 0 && joystickY > 0) { // Quadrant 1
//     _requestedAngle = (1_rad * atan2(joystickY, joystickX));
//   } else if (joystickX < 0 && joystickY > 0) { // Quadrant 2
//     _requestedAngle = 180_deg - (1_rad * atan2(joystickY, joystickX));
//   } else if (joystickX < 0 && joystickY < 0) { // Quadrant 3
//     _requestedAngle = 180_deg + (1_rad * atan2(joystickY, joystickX));
//   } else if (joystickX > 0 && joystickY < 0) { // Quadrant 4
//     _requestedAngle = 360_deg - (1_rad * atan2(joystickY, joystickX));
//   } if (joystickX == 0) {
//     if (joystickY > 0){   _requestedAngle = 90_deg;   }
//     else if (joystickY < 0){   _requestedAngle = 270_deg;   }
//   } if (joystickY == 0){
//     if (joystickX > 0){   _requestedAngle = 0_deg;   }
//     else if (joystickX < 0){   _requestedAngle = 180_deg;   }
//   }
//   // else, default to currentAngle
// }




// Code for Drivebase Pose Controls
DrivebasePoseBehaviour::DrivebasePoseBehaviour(
    wom::SwerveDrive *swerveDrivebase, frc::Pose2d pose, units::volt_t voltageLimit, bool hold)
    : _swerveDrivebase(swerveDrivebase), _pose(pose), _voltageLimit(voltageLimit), _hold(hold) {
  Controls(swerveDrivebase);
}
void DrivebasePoseBehaviour::OnTick(units::second_t deltaTime) {
  double currentAngle = _swerveDrivebase->GetPose().Rotation().Degrees().value();
  units::degree_t adjustedAngle = 1_deg * (currentAngle - fmod(currentAngle, 360) + _pose.Rotation().Degrees().value());
  _swerveDrivebase->SetVoltageLimit(_voltageLimit);
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
AlignDrivebaseToNearestGrid::AlignDrivebaseToNearestGrid(wom::SwerveDrive *swerveDrivebase) : _swerveDrivebase(swerveDrivebase){   Controls(swerveDrivebase);   }
AlignDrivebaseToNearestGrid::AlignDrivebaseToNearestGrid(wom::SwerveDrive *swerveDrivebase, Vision *vision, int alignType) : _swerveDrivebase(swerveDrivebase), _vision(vision), _alignType(alignType) {   Controls(swerveDrivebase);   }

void AlignDrivebaseToNearestGrid::OnTick(units::second_t deltaTime){

  photonlib::PhotonPipelineResult capturedImage = _vision->GetLatestResults(_vision->GetConfig()->camera);
  
  frc::Pose2d currentPose; 
  if (capturedImage.HasTargets() == false){
    currentPose = _swerveDrivebase->GetPose();
  } else {

    // if has april tag in field:
    currentPose = _vision->EstimatePose(_vision->GetConfig()).ToPose2d();
    // else
      //SetDone();
    // if (PoseNotFound) {   SetDone();   }


    frc::Pose2d nearestGrid = _gridPoses[0];
    int poseIndex = 0;
    units::degree_t alignAngle = 0_deg;
    double angle = std::fmod(currentPose.Rotation().Degrees().value(), 360);
    if (90 < angle && angle <= 270){   alignAngle = 180_deg;   }

    int iterationNum = 0;
    for (frc::Pose2d pose : _gridPoses) {
      frc::Pose2d difference = currentPose.RelativeTo(pose);
      double distance = pow(difference.X().value(), 2) + pow(difference.Y().value(), 2);
      if (distance < pow(nearestGrid.X().value(), 2) + pow(nearestGrid.Y().value(), 2)){
        nearestGrid = pose;
        poseIndex = iterationNum;
      }
      iterationNum++;
    }

    frc::Pose2d targetGridPose = nearestGrid;
    _vision->table->GetEntry("x1").SetDouble(targetGridPose.X().value());
    _vision->table->GetEntry("y1").SetDouble(targetGridPose.Y().value());



    if (_alignType == -1) { // left
      if (poseIndex != 0 & poseIndex != 9){
        targetGridPose = _gridPoses[poseIndex - 1];
      }
    }
    if (_alignType == 0) { // forward

    }
    if (_alignType == 1) { // right
      if (poseIndex != 8 & poseIndex != 17){
        targetGridPose = _gridPoses[poseIndex + 1];
      }
    }

    _vision->table->GetEntry("x2").SetDouble(targetGridPose.X().value());
    _vision->table->GetEntry("y2").SetDouble(targetGridPose.Y().value());

    if (pow(targetGridPose.X().value(), 2) + pow(targetGridPose.Y().value(), 2) < alignmentAllowDistance.value()){
      // _swerveDrivebase->SetPose(frc::Pose2d{targetGridPose.X(), targetGridPose.Y(), alignAngle});
      _vision->table->GetEntry("goToPoseX").SetDouble(targetGridPose.X().value());
      _vision->table->GetEntry("goToPoseY").SetDouble(targetGridPose.Y().value());
      _vision->table->GetEntry("goToPoseRotation").SetDouble(alignAngle.value());

      _swerveDrivebase->SetPose(frc::Pose2d{_swerveDrivebase->GetPose().X(), targetGridPose.Y(), alignAngle});
    }
    _vision->table->GetEntry("isworking").SetBoolean(true);
    std::cout << "running vision" << std::endl;
    }



    // if (_swerveDrivebase->IsAtSetPose()){   SetDone();   }
    // SetDone();
  
}

void AlignDrivebaseToNearestGrid::OnStart() { 
  std::cout << "vision started" << std::endl;
}
