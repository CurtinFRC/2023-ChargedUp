#pragma once

#include "behaviour/HasBehaviour.h"
#include "Encoder.h"
#include "Gearbox.h"
#include "PID.h"

#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <units/mass.h>
#include <units/voltage.h>

namespace wom {
  struct ArmConfig {
    wom::Gearbox gearbox;
    frc::DigitalInput *lowerLimitSwitch;
    wom::PIDConfig<units::radian, units::volt> pidConfig;

    units::kilogram_t mass;
    units::meter_t armLength;
    units::radian_t maxAngle = 180_deg;
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
  private:
    ArmConfig _config;
    ArmState _state = ArmState::kIdle;
    wom::PIDController<units::radian, units::volt> _pid;
  };

  /* SIMULATION */

  namespace sim {
    class ArmSim {
    public:
      ArmSim(ArmConfig config);

      void Update(units::volt_t voltage, units::second_t dt);

      units::radian_t angle{0};
    private:
      wom::DCMotor motor;
      units::newton_meter_t nominalTorque;
      units::radian_t minAngle, maxAngle;

      std::shared_ptr<SimCapableEncoder> encoder;
      frc::sim::DIOSim *lowerLimit;
    };
  }
}