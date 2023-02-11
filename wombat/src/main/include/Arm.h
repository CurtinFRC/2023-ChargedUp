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
  //stes up the config information 
  struct ArmConfig {
    std::string path;

    wom::Gearbox gearbox;
    wom::PIDConfig<units::radian, units::volt> pidConfig;

    units::kilogram_t armMass;
    units::kilogram_t loadMass;
    units::meter_t armLength;
    units::radian_t minAngle = -90_deg;
    units::radian_t maxAngle = 270_deg;
    units::radian_t initialAngle = 90_deg;
    units::radian_t angleOffset = 0_deg;

    void WriteNT(std::shared_ptr<nt::NetworkTable> table);
  };

  //creates all states that will be used
  enum class ArmState {
    kIdle,
    kAngle,
    kRaw
  };

  //allows the different states to be usable
  class Arm : public behaviour::HasBehaviour {
  public:
    Arm(ArmConfig config);

    //creates functions based of off the different states with useable information
    void OnUpdate(units::second_t dt);

    void SetIdle();
    void SetAngle(units::radian_t angle);
    void SetRaw(units::volt_t voltage);

    ArmConfig &GetConfig();

    units::radian_t GetAngle() const;
    units::radians_per_second_t MaxSpeed() const;
    
    bool IsStable() const;
  private:
    //information that cannot be changed or edited by user
    ArmConfig _config;
    ArmState _state = ArmState::kIdle;
    wom::PIDController<units::radian, units::volt> _pid;
    
    std::shared_ptr<nt::NetworkTable> _table;

    units::volt_t _voltage{0};
  };
};