#pragma once

#include <frc/geometry/Pose3d.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/SimPhotonCamera.h>

#include <frc/apriltag/AprilTagFieldLayout.h>

struct VisionConfig {
  std::shared_ptr<photonlib::PhotonCamera> camera;
  units::radian_t fov;
  frc::Pose3d robotToCamera;
  std::shared_ptr<frc::AprilTagFieldLayout> layout;
};

// WPILib 2023.2.1 isn't out yet... sigh...
std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout();