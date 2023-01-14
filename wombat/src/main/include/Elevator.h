#pragma once 

#include "Gearbox.h"
#include "PID.h"
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"
#include <units/length.h>
#include <units/mass.h>

#include <frc/DigitalInput.h>
#include <frc/simulation/DIOSim.h>
#include <frc/simulation/ElevatorSim.h>
#include <networktables/NetworkTable.h>

#include <memory>

namespace wom {
  enum class ElevatorState {
    kIdle, 
    kPID,
    kZeroing,
    kManual
  };

  struct ElevatorConfig {
    std::string path;
    Gearbox gearbox;
    frc::DigitalInput *topSensor;
    frc::DigitalInput *bottomSensor;
    units::meter_t radius;
    units::kilogram_t mass;
    units::meter_t maxHeight;
    units::meter_t initialHeight;
    PIDConfig<units::meter, units::volt> pid;

    void WriteNT(std::shared_ptr<nt::NetworkTable> table);
  };

  class Elevator : public behaviour::HasBehaviour {
   public: 
    Elevator(ElevatorConfig params);

    void OnUpdate(units::second_t dt);

    void SetManual(units::volt_t voltage);
    void SetPID(units::meter_t height);
    void SetZeroing();
    void SetIdle();

    bool IsStable() const;
    ElevatorState GetState() const;

    units::meter_t GetHeight() const;
    units::meters_per_second_t MaxSpeed() const;
  
   private:
    units::volt_t _setpointManual{0};

    ElevatorConfig _config;
    ElevatorState _state;

    PIDController<units::meter, units::volt> _pid;

    std::shared_ptr<nt::NetworkTable> _table;
  };

  /* SIMULATION */

  namespace sim {
    class ElevatorSim {
     public:
      ElevatorSim(ElevatorConfig config);

      void Update(units::second_t dt);

      units::meter_t GetHeight() const;
      units::ampere_t GetCurrent() const;
     private:
      ElevatorConfig config;
      frc::sim::ElevatorSim sim;

      std::shared_ptr<SimCapableEncoder> encoder;
      frc::sim::DIOSim *lowerLimit, *upperLimit;

      std::shared_ptr<nt::NetworkTable> table;
    };
  }
}