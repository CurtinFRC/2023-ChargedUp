#include "behaviour/GripperBehaviour.h"

//Behaviour used in auto to control the gripper, inputs an integer for which gripper mode you want
GripperAutoBehaviour::GripperAutoBehaviour(Gripper *gripper, int out) : gripper(gripper), _out(out) {
  Controls(gripper);
}

void GripperAutoBehaviour::OnTick(units::second_t dt) {
  if (_out == 1) {
    gripper->SetOutaking(0.7);
  } else if (_out == 2){
    gripper->SetIntaking();
  } else if (_out == 3) {
    gripper->SetIdle();
    SetDone();
  }
}

//Teleop gripper behaviour
GripperBehaviour::GripperBehaviour(Gripper *gripper, frc::XboxController &codriver)
  : gripper(gripper), _codriver(codriver) {
    Controls(gripper);
  }

void GripperBehaviour::OnTick(units::second_t dt) {
  //setting the gripper mode
  if (_codriver.GetLeftTriggerAxis() > 0.2) {
    double outtakeAmount = _codriver.GetLeftTriggerAxis() * _codriver.GetLeftTriggerAxis() * _codriver.GetLeftTriggerAxis();
    gripper->SetOutaking(outtakeAmount);
  } else if (_codriver.GetRightTriggerAxis() > 0.2) {
    gripper->SetIntaking();
  } else {
    if (idleHold) {
      gripper->SetIdle();
    }
    else {
      gripper->SetStop();
    }
  }

  if (_codriver.GetBackButtonPressed()) {
    idleHold = !idleHold;
  }
}