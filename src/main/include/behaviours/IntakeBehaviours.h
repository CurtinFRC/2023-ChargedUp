#pragma once 

#include "Gearbox.h"
#include <frc/DigitalInput.h>
#include <frc/XboxController.h>
#include "behaviour/Behaviour.h"
#include "Intake.h"

class IntakeBehaviour : public behaviour::Behaviour {
  public :
    IntakeBehaviour(Intake *intake, frc::XboxController *coDriver);

    void OnTick(units::second_t dt) override;

  private :
    Intake *_intake;
    frc::XboxController *_coDriver;
};