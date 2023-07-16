#pragma once

#include "behaviour/HasBehaviour.h"
#include "LimelightHelpers.h"
#include "Vision.h"

#include <string>
#include <vector>
#include <memory>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/json.h>
#include <frc/geometry/Pose3d.h>

class Limelight : public behaviour::HasBehaviour{
 public:
  Limelight(std::string *_limelightName): _limelightName(_limelightName) {};
  ~Limelight();

  std::string &GetName();

  std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("Limelight-Vision");

  void OnUpdate();
  void OnStart();

 private:
  std::string *_limelightName;
};