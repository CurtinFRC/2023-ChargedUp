#include "LimelightVision.h"

std::string *Limelight::GetName() { return _limelightName; }

std::pair<double, double> Limelight::GetOffset() {
  std::pair<double, double> offset;
  
  offset.first = table->GetNumber("tx",0.0);
  offset.second = table->GetNumber("ty",0.0);

  return offset;
}

std::vector<double> Limelight::GetAprilTagData(std::string dataName) {
  return table->GetNumberArray(dataName ,std::vector<double>(6));
}

units::meters_per_second_t Limelight::GetSpeed(frc::Pose3d pose1, frc::Pose3d pose2, units::second_t dt) {
  // ask anna
  frc::Transform3d dPose{pose1, pose2}; 
  frc::Translation3d dTranslation = dPose.Translation();

  units::meter_t y = dTranslation.Y();
  units::meter_t x = dTranslation.X();
  units::radian_t theta = units::math::atan(y / x);
  units::meter_t dTRANSLATION = x / units::math::cos(theta);
  return units::math::fabs(dTRANSLATION / dt);
}

frc::Pose3d Limelight::GetPose() {
  std::vector<double> pose = GetAprilTagData("botpose");
  return frc::Pose3d(pose[1] * 1_m, 1_m * pose[2], 1_m * pose[3], frc::Rotation3d(1_deg *(pose[4]), 1_deg *(pose[5]), 1_deg *(pose[6])));
}

void Limelight::OnStart() {
  std::cout << "starting" << std::endl;
}

void Limelight::OnUpdate(units::time::second_t dt) {
  wom::WritePose3NT(table, GetPose());
}

bool Limelight::IsAtSetPoseVision(frc::Pose3d pose, units::second_t dt)  {
  frc::Pose3d actualPose = GetPose();
  frc::Pose3d relativePose = actualPose.RelativeTo(pose);
  return (units::math::fabs(relativePose.X()) < 0.01_m && units::math::fabs(relativePose.Y()) < 0.01_m && GetSpeed(pose, GetPose(), dt) < 1_m / 1_s);
}
