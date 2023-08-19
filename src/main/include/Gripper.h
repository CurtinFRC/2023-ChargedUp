#pragma once 

#include "behaviour/HasBehaviour.h"
#include <string>
#include <units/math.h>
#include <units/velocity.h>
#include <units/charge.h>
#include "Gearbox.h"


struct GripperConfig {
  wom::MotorVoltageController *gripperMotor;
};

enum class GripperState {
  kIdle,
  kIntaking,
  kOutaking,
  kHolding,
  kStop
};

class Gripper : public behaviour::HasBehaviour {
 public: 
  Gripper(GripperConfig config);

  void OnUpdate(units::second_t dt);

  void SetIdle();
  void SetIntaking(double speed);
  void SetOutaking(double speed);
  void SetHolding();
  void SetStop();

  std::string GetState();

 private: 
  double _speed;
  GripperConfig _config;
  GripperState _state = GripperState::kIdle;
};
