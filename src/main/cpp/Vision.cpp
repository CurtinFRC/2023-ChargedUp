#include "Vision.h"

#include <wpi/json.h>

#include <frc/apriltag/AprilTagFields.h>

std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout() {
  return std::make_shared<frc::AprilTagFieldLayout>(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp));
}

//unnecessary I believe but was in code previously

// Vision::Vision(VisionConfig config) 
//   : visionConfig(config),
//   _estimator(
//     config.layout,
//     photonlib::AVERAGE_BEST_TARGETS,
//     {
//       std::make_pair(config.camera, config.robotToCamera)
//     }
//   )
// { }

void Vision::OnUpdate(units::second_t dt) {
  
}

