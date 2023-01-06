#pragma once
#include "Gearbox.h"
#include <frc/DigitalInput.h>
#include "behaviour/HasBehaviour.h"

struct ClimberConfig{
    wom::Gearbox gearbox;

    units::volt_t windupVoltage = 7_V;
    units::volt_t winddownVoltage = -5_V;
};
enum class ClimberState {
  kIdle,
  kWindUp,
  kWindDown,
  kLocked
};

class Climber : public behaviour::HasBehaviour {
 public:
  Climber(ClimberConfig config);

  void OnUpdate(units::second_t dt);
  
  void SetWindUp();
  void SetWindDown();
  void SetLocked();
  void SetIdle();

  ClimberState GetState() const;

 private:
  ClimberConfig _config;
  ClimberState _state = ClimberState::kIdle;
  };