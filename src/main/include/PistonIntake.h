#pragma once

#include "Gearbox.h"
#include "behaviour/HasBehaviour.h"
#include <frc/DoubleSolenoid.h>


struct PistonIntakeConfig {
  frc::DoubleSolenoid *solenoid;
};

enum class PistonIntakeState {
  kIdle,
  kIntaking,
  kOuttaking
};

class PistonIntake : public behaviour::HasBehaviour {
 public:
  PistonIntake(PistonIntakeConfig config);

  void OnUpdate(units::second_t dt);

  void SetIdle();
  void SetIntaking();
  void SetOuttaking();

  PistonIntakeState GetState() const;

 private:
  PistonIntakeConfig _config;
  PistonIntakeState _state = PistonIntakeState::kIdle;
};