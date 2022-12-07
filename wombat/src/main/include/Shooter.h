#pragma once

#include "Gearbox.h"
#include "PID.h"
#include "behaviour/HasBehaviour.h"

#include <networktables/NetworkTable.h>
#include <units/angular_velocity.h>
#include <units/charge.h>

#include <memory>

namespace wom {
  enum class ShooterState {
    kPID,
    kManual,
    kIdle
  };

  struct ShooterParams {
    Gearbox gearbox;
    PIDConfig<units::radians_per_second, units::volt> pid;
    units::ampere_t currentLimit;
  };

  class Shooter : public behaviour::HasBehaviour {
   public:
    Shooter(ShooterParams params);

    void SetManual(units::volt_t voltage);
    void SetPID(units::radians_per_second_t goal);
    void SetIdle();

    void OnUpdate(units::second_t dt);

    bool IsStable() const;

   private:
    ShooterParams _params;
    ShooterState _state;

    units::volt_t _setpointManual{0};

    PIDController<units::radians_per_second, units::volt> _pid;

    std::shared_ptr<nt::NetworkTable> _table;
  };
}
