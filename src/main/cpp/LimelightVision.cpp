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

void Limelight::OnStart() {
  std::cout << "starting" << std::endl;
}

void Limelight::OnUpdate(units::time::second_t dt) {
  std::vector<double> pose = GetAprilTagData("botpose");
  frc::Pose3d _pose = frc::Pose3d(pose[1] * 1_m, 1_m * pose[2], 1_m * pose[3], frc::Rotation3d(1_deg *(pose[4]), 1_deg *(pose[5]), 1_deg *(pose[6])));
  wom::WritePose3NT(table, _pose);
}
