#pragma once

#include "Gearbox.h"
#include "behaviour/HasBehaviour.h"
#include <frc/DigitalInput.h>
#include "Encoder.h"
#include "PID.h"

struct ArmConfig {
  wom::Gearbox gearbox;
  frc::DigitalInput *bottomLimitSwitch;
  /*configers pid*/
  wom::PIDConfig<units::radian, units::volt> pidConfig;
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
  /*set up angle*/
  void SetAngle(units::radian_t angle);
  void SetZeroing();
 private:
  ArmConfig _config;
  ArmState _state = ArmState::kIdle;
  /*sets the paramiters of the pid: radians and volts*/
  wom::PIDController<units::radian, units::volt> _pid;
};

/* SIMULATION */
#include <units/mass.h>
#include <units/voltage.h>

namespace sim {
  class ArmSim {
   public:
    ArmSim(wom::DCMotor motor, units::kilogram_t mass, units::meter_t armLength);

    void Update(units::volt_t voltage, units::second_t dt);

    bool IsLimit() const;

    units::radian_t angle{0};
   private:
    units::newton_meter_t nominalTorque;
    bool isLimitTriggered = false;
    wom::DCMotor motor;
  };
}
