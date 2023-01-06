#pragma once

#include "Gearbox.h"
#include <frc/DigitalInput.h>

#include "behaviour/HasBehaviour.h"

struct IntakeConfig {
  wom::Gearbox gearbox;
  frc::DigitalInput *gearPresenceSensor;

  units::volt_t IntakeVoltage = 10_V;
  units::volt_t OuttakeVoltage = -7_V;
  units::volt_t holdVoltage = 2_V;
};

enum class IntakeState {
  kIdle,
  kIntaking,
  kFull,
  kOuttaking
};

class Intake : public behaviour::HasBehaviour {
 public:
  Intake(IntakeConfig config);

  void OnUpdate(units::second_t dt);
  
  void SetIntaking();
  void SetOuttaking();
 private:
  IntakeConfig _config;
  IntakeState _state = IntakeState::kIdle;
};