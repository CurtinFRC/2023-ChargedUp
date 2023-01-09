#pragma once

#include "behaviour/HasBehaviour.h"
#include "Encoder.h"
#include "Gearbox.h"
#include "PID.h"

#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <units/mass.h>
#include <units/voltage.h>
#include <units/current.h>

namespace wom {
  struct ArmConfig {
    std::string path;

    wom::Gearbox gearbox;
    frc::DigitalInput *lowerLimitSwitch;
    frc::DigitalInput *upperLimitSwitch;
    wom::PIDConfig<units::radian, units::volt> pidConfig;

    units::kilogram_t mass;
    units::meter_t armLength;
    units::radian_t minAngle = 0_deg;
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

      void Update(units::second_t dt);

      units::ampere_t GetCurrent() const;
    private:
      ArmConfig config;

      units::radians_per_second_t velocity{0};
      units::radian_t angle{0};
      units::ampere_t current{0};

      std::shared_ptr<SimCapableEncoder> encoder;
      frc::sim::DIOSim *lowerLimit, *upperLimit;

      std::shared_ptr<nt::NetworkTable> table;
    };
  }
}