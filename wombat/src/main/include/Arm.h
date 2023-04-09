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

    wom::Gearbox leftGearbox;
    wom::Gearbox rightGearbox;
    rev::SparkMaxRelativeEncoder armEncoder;
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
    kRaw
  };

  class Arm : public behaviour::HasBehaviour {
  public:
    Arm(ArmConfig config);

    void OnUpdate(units::second_t dt);

    void SetIdle();
    void SetAngle(units::radian_t angle);
    void SetRaw(units::volt_t voltage);

    void SetArmSpeedLimit(double limit); //units, what are they?? 

    ArmConfig &GetConfig();

    units::radian_t GetAngle() const;
    units::radians_per_second_t MaxSpeed() const;
    
    bool IsStable() const;
  private:
    ArmConfig _config;
    ArmState _state = ArmState::kIdle;
    wom::PIDController<units::radian, units::volt> _pid;
    
    std::shared_ptr<nt::NetworkTable> _table;

    double armLimit = 0.4;

    units::volt_t _voltage{0};
  };
};