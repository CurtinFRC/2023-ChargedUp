#pragma once 

#include "Gearbox.h"
#include "PID.h"
#include "behaviour/HasBehaviour.h"
#include "behaviour/Behaviour.h"
#include <units/length.h>
#include <units/mass.h>
#include <frc/DigitalInput.h>


#include <memory>
#include <networktables/NetworkTable.h>

namespace wom {
  enum class ElevatorState {
    kIdle, 
    kPID,
    kZeroing,
    kManual
  };

  struct ElevatorParams {
    Gearbox gearbox;
    units::meter_t radius;
    units::kilogram_t mass;
    frc::DigitalInput *topSensor;
    frc::DigitalInput *bottomSensor;
    PIDConfig<units::meter, units::volt> pid;
  };

  class Elevator : public behaviour::HasBehaviour {
   public: 
    Elevator(std::string path, ElevatorParams params);

    void OnUpdate(units::second_t dt);

    void SetManual(units::volt_t voltage);
    void SetPID();
    void SetZeroing();
    void SetIdle();

    bool IsStable() const;
    ElevatorState GetState() const;
  
   private:
    units::volt_t _setpointManual{0};

    ElevatorParams _params;
    ElevatorState _state;

    PIDController<units::meter, units::volt> _pid;

    std::shared_ptr<nt::NetworkTable> _table;
  };
}