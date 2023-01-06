#pragma once

#include "Gearbox.h"
#include "behaviour/HasBehaviour.h"
#include <frc/DigitalInput.h>
#include "Encoder.h"

struct ArmConfig {
  wom::Gearbox gearbox;
  frc::DigitalInput *bottomLimitSwitch;
};

enum class ArmState {
  kIdle,
  kAngle,
  kZeroing
};

class Arm : public behaviour::HasBehaviour {
 public:
  Arm(ArmConfig config);

  void OnUpdate(units::second_t dt);

  void SetIdle();
  void SetAngle(units::radian_t angle);
  void SetZeroing();

  ArmState GetState() const;
 private:
  ArmConfig _config;
  ArmState _state = ArmState::kIdle;
  units::radian_t _targetAngle;
};