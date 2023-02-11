#include "behaviour/GripperBehaviour.h"

GripperBehaviour::GripperBehaviour(Gripper *gripper, frc::XboxController &codriver)
  : gripper(gripper), _codriver(codriver) {
    Controls(gripper);
  }

void GripperBehaviour::OnStart() {

}

void GripperBehaviour::OnTick(units::second_t dt) {

  if (_codriver.GetYButtonReleased()) {
    if (holdingObject) {
      holdingObject = false;
    } else {
      holdingObject = true;
    }
  }

  if (holdingObject) {
    gripper->SetHolding();
  } else {
    if (_codriver.GetRightBumper()) {
      gripper->SetIntaking();
    } else if (_codriver.GetLeftBumper()) {
      gripper->SetOutaking();
    } else {
      gripper->SetIdle();
    }
  }
}