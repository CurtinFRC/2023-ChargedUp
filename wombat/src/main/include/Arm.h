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

    units::kilogram_t armMass;
    units::kilogram_t loadMass;
    units::meter_t armLength;
    units::radian_t minAngle = 0_deg;
    units::radian_t maxAngle = 180_deg;
    units::radian_t initialAngle = 0_deg;
    units::radian_t angleOffset = 0_deg;

    void WriteNT(std::shared_ptr<nt::NetworkTable> table);
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

    ArmConfig &GetConfig();

    units::radian_t GetAngle() const;
    units::radians_per_second_t MaxSpeed() const;
    
    bool IsStable() const;
  private:
    ArmConfig _config;
    ArmState _state = ArmState::kIdle;
    wom::PIDController<units::radian, units::volt> _pid;
    
    std::shared_ptr<nt::NetworkTable> _table;
  };

  /* SIMULATION */

  namespace sim {
    class ArmSim {
    public:
      ArmSim(ArmConfig config);

      void Update(units::second_t dt);

      units::ampere_t GetCurrent() const;

      ArmConfig config;

      units::newton_meter_t torque{0};
      units::newton_meter_t additionalTorque{0};
      units::radians_per_second_t velocity{0};
      units::radian_t angle{0};
      units::ampere_t current{0};

      std::shared_ptr<SimCapableEncoder> encoder;
      frc::sim::DIOSim *lowerLimit, *upperLimit;

      std::shared_ptr<nt::NetworkTable> table;
    };
  }
}