#include "Vision.h"
#include <networktables/NetworkTableInstance.h>


Vision::Vision() {}

void Vision::Update(units::second_t dt) {
  // photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  double distance = _visionTable->GetEntry("TargetYaw").GetDouble(0);
  std::cout << distance << std::endl;
}