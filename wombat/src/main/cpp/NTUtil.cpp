#include "NTUtil.h"

void wom::WritePose2NT(std::shared_ptr<nt::NetworkTable> table, frc::Pose2d pose) {
  table->GetEntry("x").SetDouble(pose.X().value());
  table->GetEntry("y").SetDouble(pose.Y().value());
  table->GetEntry("angle").SetDouble(pose.Rotation().Degrees().value());
}

<<<<<<< HEAD
void wom::WritePose3NT(std::shared_ptr<nt::NetworkTable> table, frc::Pose3d pose) {
  table->GetEntry("x").SetDouble(pose.X().value());
  table->GetEntry("y").SetDouble(pose.Y().value());
  table->GetEntry("z").SetDouble(pose.Z().value());
=======
void wom::WritePose2NT(std::shared_ptr<nt::NetworkTable> table, frc::Pose3d pose) {
  table->GetEntry("x").SetDouble(pose.X().value());
  table->GetEntry("y").SetDouble(pose.Y().value());
  table->GetEntry("z").SetDouble(pose.Z().value());

  table->GetEntry("angle").SetDouble(pose.Rotation().Z().convert<units::degree>().value());
>>>>>>> f473d620c7e327dfeb37757cc5a8a3d6c34ac45c
}