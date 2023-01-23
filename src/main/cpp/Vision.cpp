#include "Vision.h"

#include <wpi/json.h>

#include <frc/apriltag/AprilTagFields.h>

std::shared_ptr<frc::AprilTagFieldLayout> Get2023Layout() {
  return std::make_shared<frc::AprilTagFieldLayout>(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp));
}