#include "behaviour/LimelightBehaviours.h"


AlignBehaviour::AlignBehaviour(Limelight *limelight, frc::XboxController &driver, wom::SwerveDrive *swerve)
    : _limelight(limelight), _driver(driver), _swerve(swerve) {
  Controls(limelight);
  Controls(swerve);
}

void AlignBehaviour::AlignToTarget(frc::Pose3d pose) {
  //swerve->SetPose(pose);

  _state = poseState::notAtPose;
  _setPoint = pose;
}

void AlignBehaviour::CancelAlign() {
  _state = poseState::atPose;
}

void AlignBehaviour::OnTick(units::second_t dt) {
    if (_driver.GetPOV() == 0) {
        AlignToTarget(_SetPoints[0]);
    } else if (_driver.GetPOV() == 90) {
        AlignToTarget(_SetPoints[1]);
    } else if (_driver.GetPOV() == 180) {
        AlignToTarget(_SetPoints[2]);
    } else if (_driver.GetPOV() == 270) {
        AlignToTarget(_SetPoints[3]);
    } else if (_driver.GetBackButtonPressed()) {
        CancelAlign();
    }

    if (_state == poseState::notAtPose && !_limelight->IsAtSetPoseVision(_setPoint, dt)) {
        _swerve->SetPose(_setPoint.ToPose2d());
    }
    else {
        _state = poseState::atPose;
    }
}