#pragma once 
#include <networktables/NetworkTableInstance.h>
#include <iostream>
#include <units/length.h>
#include <units/time.h>


// struct VisionConfig {
//   units::meter_t offset_x;
//   units::meter_t offset_y;
// };

class Vision {
 public: 
  Vision();
  void Update(units::second_t dt);

 private: 
  // photonlib::PhotonCamera camera{"photonvision"};
  std::shared_ptr<nt::NetworkTable> _visionTable = nt::NetworkTableInstance::GetDefault().GetTable("photonvision/visionCam");

  // VisionConfig _config;
};