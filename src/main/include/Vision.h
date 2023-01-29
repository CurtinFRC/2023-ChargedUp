#pragma once

#include <frc/geometry/Transform3d.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/RobotPoseEstimator.h>
#include <photonlib/SimPhotonCamera.h>

// #include <frc/geometry/Pose3d.h>
// #include <photonlib/PhotonCamera.h>
// #include <photonlib/SimPhotonCamera.h>

#include <math.h>
#include <iostream>

struct VisionConfig {
  std::shared_ptr<photonlib::PhotonCamera> camera;
  frc::Transform3d robotToCamera;
  std::shared_ptr<frc::AprilTagFieldLayout> layout;
};

std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout();

class Vision {
  public :
    Vision(VisionConfig config);
    void OnUpdate(units::second_t dt);   

  private :
    VisionConfig _config;
    photonlib::RobotPoseEstimator _estimator;
};
