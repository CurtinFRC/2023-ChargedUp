#include "Vision.h"

<<<<<<< HEAD
// initializer for estimator
using map_value_type = std::pair<std::shared_ptr<PhotonCamera>, frc::Transform3d>;
Vision::Vision(VisionConfig config) 
  : visionConfig(config),
  _estimator(RobotPoseEstimator{
    config.layout,
    photonlib::AVERAGE_BEST_TARGETS,
    {std::make_pair(config.camera, config.robotToCamera)}
    }

    ),
  defaultConfig(config)
{ };
=======
std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout() {
  return std::make_shared<frc::AprilTagFieldLayout>(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp));
}

Vision::Vision(VisionConfig *config) 
  : _config(config),
  _estimator(
    config->layout,
    photonlib::AVERAGE_BEST_TARGETS,
    {
      std::make_pair(config->camera, config->robotToCamera)
    }
  )
{ }

std::optional<std::pair<frc::Pose3d, units::second_t>> Vision::OnUpdate(units::second_t dt) {
  //photonlib::PhotonPipelineResult result = _config.camera->GetLatestResult();
  // photonlib::PhotonTrackedTarget target = result.GetBestTarget();
  std::pair<frc::Pose3d, units::millisecond_t> pose_result = _estimator.Update();
  auto table = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
  wom::WritePose2NT(table, pose_result.first);
  if (pose_result.second > 0_ms) {  return std::make_pair(pose_result.first, wom::now() - pose_result.second);  }
  else {  return {};  }
}
>>>>>>> f473d620c7e327dfeb37757cc5a8a3d6c34ac45c
