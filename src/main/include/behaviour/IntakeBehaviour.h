#pragma once

#include "RobotMap.h"
#include "behaviour/Behaviour.h"
#include "Intake.h"

#include <frc/XboxController.h>

class IntakeAutoBehaviour : public behaviour::Behaviour {
 public: 
  IntakeAutoBehaviour(Intake *intake, int out);

  void OnTick(units::second_t dt) override;
 private:
  Intake *intake;
  int _out;
};

class IntakeBehaviour : public behaviour::Behaviour {
  public:
    IntakeBehaviour(Intake *intake, frc::XboxController *codriver);
  
    void OnTick(units::second_t dt) override;

  private:
    Intake *intake;
    frc::XboxController *_codriver;

    
};

