#pragma once 

#include "RobotMap.h"
#include "behaviour/Behaviour.h"
#include "Gripper.h"

class GripperAutoBehaviour : public behaviour::Behaviour {
 public: 
  GripperAutoBehaviour(Gripper *gripper, int out);

  void OnTick(units::second_t dt) override;
 private:
  Gripper *gripper;
  int _out;
};

class GripperBehaviour : public behaviour::Behaviour {
 public: 
  GripperBehaviour(Gripper *gripper, frc::XboxController &codriver);

  void OnTick(units::second_t dt) override;

 private: 
  Gripper *gripper;
  frc::XboxController &_codriver;

  bool holdingObject = false;

  bool idleHold = false;
};
