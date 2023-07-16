#include "LimelightVision.h"

std::string *Limelight::GetName() { return _limelightName; }

std::pair<double, double> Limelight::GetOffset() {
  std::pair<double, double> offset;
  
  offset.first = table->GetNumber("tx",0.0);
  offset.second = table->GetNumber("ty",0.0);

  return offset;
}

auto Limelight::GetAprilTagData(std::string dataName) {
  return table->GetNumberArray(dataName ,std::vector<double>(6));
}

void Limelight::OnStart() {
  std::cout << "starting" << std::endl;
}

void Limelight::OnUpdate(units::time::second_t dt) {
  auto pose = GetAprilTagData("botpose");
  // frc::Pose3d _pose = frc::Pose3d(units::meter{pose[1]}, units::meter{pose[2]}, units::meter{pose[3]}, frc::Rotation3d(units::degree{pose[4]}, units::degree{pose[5]}, units::degree{pose[6]}));
  frc::Pose3d _pose = frc::Pose3d(units::meter(pose[1]), units::meter(pose[2]), units::meter(pose[3]), frc::Rotation3d(units::degree(pose[4]), units::degree(pose[5]), units::degree(pose[6])));
  wom::WritePose3NT(table, _pose);
}
