#pragma once

#include "behaviour/HasBehaviour.h"
#include "Vision.h"

#include <string>
#include <vector>
#include <memory>
#include <iostream>
#include <utility>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/json.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <units/angle.h>
#include <units/length.h>

class Limelight : public behaviour::HasBehaviour{
 public:
  Limelight(std::string *_limelightName): _limelightName(_limelightName) {};
  ~Limelight();

  std::string *GetName();

  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  std::pair<double, double> GetOffset();
  
  auto GetAprilTagData(std::string dataName);

  void OnUpdate(units::time::second_t dt);
  void OnStart();

 private:
  std::string *_limelightName;
};
