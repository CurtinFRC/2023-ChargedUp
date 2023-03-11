#pragma once

#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Pose3d.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/SimPhotonCamera.h>
#include <photonlib/RobotPoseEstimator.h>
#include <NTUtil.h>

#include <frc/apriltag/AprilTagFieldLayout.h>

using namespace photonlib;

// pp stands for photonpipeline in variable naming

struct VisionConfig {
  //current master version of camera declaration
  std::shared_ptr<PhotonCamera> camera;

  //current version
  // both declaration versions work but you need to use -> for std pointer
  // PhotonCamera camera;

  units::radian_t fov;
  frc::Transform3d robotToCamera;
  std::shared_ptr<frc::AprilTagFieldLayout> layout;


};

class Vision {
  private :
    VisionConfig visionConfig;
    photonlib::RobotPoseEstimator _estimator;

  public :
    Vision(VisionConfig config, RobotPoseEstimator estimator);
    
    void OnUpdate(units::second_t dt); 

    PhotonPipelineResult getLatestResults(std::shared_ptr<PhotonCamera> camera) {
      PhotonPipelineResult ppResult = camera->GetLatestResult();
      return ppResult;
    };

    auto getTargets(std::shared_ptr<PhotonCamera> camera) {
      PhotonPipelineResult ppResults = getLatestResults(visionConfig.camera);
      auto targets = ppResults.GetTargets();
      return targets;
    };

  PhotonTrackedTarget getBestTarget(std::shared_ptr<PhotonCamera> camera, PhotonPipelineResult result) {
    PhotonTrackedTarget bestTarget = result.GetBestTarget();
    return bestTarget;
  };

  auto estimatePose() {
    std::pair<frc::Pose3d, units::millisecond_t> pose_result = _estimator.Update();
    auto table = nt::NetworkTableInstance::GetDefault().GetTable("Vision");
    wom::WritePose3NT(table, pose_result.first);
    return pose_result.first;
  };

  auto getPathForBest(std::shared_ptr<PhotonCamera> camera) {
    PhotonPipelineResult ppResults = getLatestResults(camera);
    PhotonTrackedTarget bestTarget = getBestTarget(camera, ppResults);
    auto poseEstimate = estimatePose();
    auto bestTargetPose = estimatePose();
    auto unrefinedPath =  - pose_result.first;

  };

};

  // //new other things
  // PhotonPipelineResult ppResult = camera.GetLatestResult();

  // bool hasTargets = ppResult.HasTargets();

  // //PhotonTrackedTarget targets = ppResult.GetTargets();

  // auto targets = ppResult.GetTargets();
  
  // PhotonTrackedTarget target = ppResult.GetBestTarget();



std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout();