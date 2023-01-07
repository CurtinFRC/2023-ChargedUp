#include "behaviours/IntakeBehaviours.h"

IntakeBehaviour::IntakeBehaviour(Intake *intake, frc::XboxController *coDriver) : _intake(intake), _coDriver(coDriver) {
  Controls(intake);
}

void IntakeBehaviour::OnTick(units::second_t dt) {
  if (_coDriver->GetRightBumper()) {
    _intake->SetIntaking();
  } else if (_coDriver->GetLeftBumper()) {
    _intake->SetOuttaking();
  } else {
    _intake->SetIdle();
  }
  std::cout << "in tick" << std::endl;
}