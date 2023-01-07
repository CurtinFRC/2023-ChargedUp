#include "behaviours/ClimberBehaviour.h"

ClimberBehaviour::ClimberBehaviour(Climber *climber,frc::XboxController *CoDriver):_climber(climber),_codriver(CoDriver){
    Controls(climber);
}

void ClimberBehaviour::OnTick(units::second_t dt){
    if (_codriver->GetLeftTriggerAxis()>0.05) {
        _climber->SetWindUp();
    } else if (_codriver->GetRightTriggerAxis()>0.05){
        _climber->SetWindDown();
    }
    if (_codriver->GetStartButtonPressed()){
        if (IsLocked){
            _climber->SetIdle();
        }else {
            _climber->SetLocked();
        }
    }
}