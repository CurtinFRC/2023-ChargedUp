#include "Vision.h"

Vision::Vision(VisionConfig *config) : _config(config), _estimator(config->layout, photonlib::AVERAGE_BEST_TARGETS, {std::make_pair(config->camera, config->robotToCamera)})
{ }

std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout() {
  return std::make_shared<frc::AprilTagFieldLayout>(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp));
};

void Vision::OnUpdate(units::second_t dt) {
  //photonlib::PhotonPipelineResult result = _config.camera->GetLatestResult();
  // photonlib::PhotonTrackedTarget target = result.GetBestTarget();
  std::pair<frc::Pose3d, units::millisecond_t> pose_result = _estimator.Update();
  
  wom::WritePose3NT(table, pose_result.first);
}

bool Vision::IsAtSetPoseVision(VisionConfig _config, frc::Pose2d targetPosition)  {
  auto currentPose = EstimatePose(_config);
  frc::Pose2d currentPose2d{currentPose.X(),currentPose.Y(), currentPose.Rotation().ToRotation2d()};
  if (currentPose2d == targetPosition) {return true;} else {return false;}
}