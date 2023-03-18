#include "Vision.h"

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

void Vision::OnUpdate(units::second_t dt) {
  
}