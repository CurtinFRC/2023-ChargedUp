#pragma once 

#include "behaviour/HasBehaviour.h"
#include <string>
#include <units/math.h>
#include <units/velocity.h>
#include <units/charge.h>
// #include "MotorVoltageController.h"
#include "Gearbox.h"


struct GripperConfig {
  wom::MotorVoltageController *leftGripperMotor;
  wom::MotorVoltageController *rightGripperMotor;
};

enum class GripperState {
  kIdle,
  kIntaking,
  kOutaking,
  kHolding
};

class Gripper : public behaviour::HasBehaviour {
 public: 
  Gripper(GripperConfig config);

  void OnUpdate(units::second_t dt);

  void SetIdle();
  void SetIntaking();
  void SetOutaking();
  void SetHolding();

  std::string GetState();

 private: 
  GripperConfig _config;
  GripperState _state = GripperState::kIdle;
};