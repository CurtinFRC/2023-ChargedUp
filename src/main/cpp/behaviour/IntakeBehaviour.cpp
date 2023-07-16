#include "behaviour/IntakeBehaviour.h"

//Behaviour used in auto to control the intake, inputs an integer for which intake mode you want

IntakeAutoBehaviour::IntakeAutoBehaviour(Intake *intake, int out) : intake(intake), _out(out) {
  Controls(intake);
}

void IntakeAutoBehaviour::OnTick(units::second_t dt) {

}

IntakeBehaviour::IntakeBehaviour(Intake *intake, frc::XboxController *codriver)
  : intake(intake), _codriver(codriver) {
    Controls(intake);
  }

void IntakeBehaviour::OnTick(units::second_t dt) {
    if (_codriver->GetRightY() > 0.2) {
        intake->SetIntake();
    } else if (_codriver->GetRightY() < -0.2) {
        intake->SetOuttake();
    } else {
        intake->SetIdle();
    }
}