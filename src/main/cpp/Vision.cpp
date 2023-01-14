#include "Vision.h"

/* SIMULATION */

VisionSim::VisionSim(VisionConfig config) : config(config) {}

void VisionSim::Update(units::second_t dt, frc::Pose2d realPose) {
  frc::Pose3d realPose3d{
    frc::Translation3d{ realPose.X(), realPose.Y(), 0_m },
    frc::Rotation3d{ 0_rad, 0_rad, realPose.Rotation().Radians() }
  };

  
}