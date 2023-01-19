#include "Vision.h"

#include <wpi/json.h>

#include <frc/apriltag/AprilTagFields.h>
#include "NTUtil.h"

std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout() {
  return std::make_shared<frc::AprilTagFieldLayout>(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp));
}

Vision::Vision(VisionConfig config) 
  : _config(config),
  _estimator(
    config.layout,
    photonlib::AVERAGE_BEST_TARGETS,
    {
      std::make_pair(config.camera, config.robotToCamera)
    }
  )
{ }

void Vision::OnUpdate(units::second_t dt) {
  // photonlib::PhotonPipelineResult result = _config.camera->GetLatestResult();
  // photonlib::PhotonTrackedTarget target = result.GetBestTarget();
  // important information : fudicialID, pose ambuiguity, maybe : yaw, pitch and area
  std::pair<frc::Pose3d, units::millisecond_t> result = _estimator.Update();
  auto table = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
  wom::WritePose2NT(table, result.first);
}