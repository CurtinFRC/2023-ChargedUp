#pragma once

#include <frc/geometry/Pose3d.h>
#include <photonlib/PhotonCamera.h>

#include <frc/apriltag/AprilTagFieldLayout.h>

struct VisionConfig {
  photonlib::PhotonCamera *camera;
  units::radian_t fov;
  frc::Pose3d robotToCamera;
  frc::AprilTagFieldLayout layout;
};

/* SIMULATION */

class VisionSim {
 public:
  VisionSim(VisionConfig config);

  void Update(units::second_t dt, frc::Pose2d realPose);
 private:
  VisionConfig config;
};