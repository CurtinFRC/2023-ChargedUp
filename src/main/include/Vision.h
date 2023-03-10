#pragma once

#include <frc/geometry/Pose3d.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/SimPhotonCamera.h>

#include <frc/apriltag/AprilTagFieldLayout.h>

using namespace photonlib;

// pp stands for photonpipeline in variable naming

struct VisionConfig {
  //current master version of camera declaration
  // std::shared_ptr<PhotonCamera> camera;

  //current version
  // both declaration versions work but you need to use -> for std pointer
  PhotonCamera camera;

  units::radian_t fov;
  frc::Pose3d robotToCamera;
  std::shared_ptr<frc::AprilTagFieldLayout> layout;


};

class Vision {
  private :
    VisionConfig visionConfig;

  public :
    Vision(VisionConfig config);

    PhotonPipelineResult getLatestResults(PhotonCamera &camera) {
      PhotonPipelineResult ppResult = camera.GetLatestResult();
      return ppResult;
    };

    auto getTargets(PhotonCamera &camera) {
      PhotonPipelineResult ppResults = getLatestResults(visionConfig.camera);
      auto targets = ppResults.GetTargets();
      return targets;
    };

  PhotonTrackedTarget getBestTarget(PhotonCamera &camera, PhotonPipelineResult result) {
    PhotonTrackedTarget bestTarget = result.GetBestTarget();
    return bestTarget;
  };

  auto getPathForBest(PhotonCamera &camera) {
    PhotonPipelineResult getLatestResults(PhotonCamera camera);
    PhotonTrackedTarget getBestTarget(PhotonCamera camera, PhotonPipelineResult ppResult);
    

  };

};

  // //new other things
  // PhotonPipelineResult ppResult = camera.GetLatestResult();

  // bool hasTargets = ppResult.HasTargets();

  // //PhotonTrackedTarget targets = ppResult.GetTargets();

  // auto targets = ppResult.GetTargets();
  
  // PhotonTrackedTarget target = ppResult.GetBestTarget();



std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout();