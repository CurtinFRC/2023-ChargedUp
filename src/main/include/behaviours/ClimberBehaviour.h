#pragma once
#include "Gearbox.h"
#include <frc/XboxController.h>
#include "behaviour/Behaviour.h"
#include "Climber.h"

class ClimberBehaviour : public behaviour::Behaviour {
 public:
  ClimberBehaviour (Climber *climber,frc::XboxController *CoDriver);

  void OnTick(units::second_t dt);
  
 private:
  Climber *_climber;
  bool IsLocked = false;
  frc::XboxController *_codriver;
  };