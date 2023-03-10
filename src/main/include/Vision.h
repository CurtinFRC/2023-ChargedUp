#pragma once

#include <frc/geometry/Pose3d.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/SimPhotonCamera.h>

#include <frc/apriltag/AprilTagFieldLayout.h>

// pp stands for photonpipeline in variable naming

struct VisionConfig {
  //current master version of camera declaration
  // std::shared_ptr<photonlib::PhotonCamera> camera;

  //current version
  // both declaration versions work but you need to use -> for std pointer
  photonlib::PhotonCamera camera;

  //untouched
  units::radian_t fov;
  frc::Pose3d robotToCamera;
  std::shared_ptr<frc::AprilTagFieldLayout> layout;

  //new other things
  photonlib::PhotonPipelineResult ppResult = camera.GetLatestResult();
};

std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout();