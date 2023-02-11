#pragma once

#include "behaviour/HasBehaviour.h"

#include <frc/geometry/Transform3d.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/RobotPoseEstimator.h>
#include <photonlib/SimPhotonCamera.h>
// // #include "drivetrain/SwerveDrive.h"

#include <frc/geometry/Pose3d.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/SimPhotonCamera.h>

#include <math.h>
#include <iostream>

#include <wpi/json.h>
#include <units/length.h>
#include <frc/apriltag/AprilTagFields.h>
#include "NTUtil.h"
#include "Util.h"

struct VisionConfig {
  std::shared_ptr<photonlib::PhotonCamera> camera;
  frc::Transform3d robotToCamera;
  std::shared_ptr<frc::AprilTagFieldLayout> layout;
};

std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout();

class Vision : public behaviour::HasBehaviour {
  public:
    Vision(VisionConfig *config);
    std::optional<std::pair<frc::Pose3d, units::second_t>> OnUpdate(units::second_t dt);   

    void OnTick(units::second_t dt); 

  private:
    VisionConfig *_config;
    photonlib::RobotPoseEstimator _estimator;
};
