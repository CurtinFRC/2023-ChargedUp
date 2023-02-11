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
  //creates states used for the elevator
  enum class ElevatorState {
    kIdle, 
    kPID,
    kManual,
    kRaw
  };

  //creates infromation that is used in the config
  struct ElevatorConfig {
    std::string path;
    wom::Gearbox gearbox;
    frc::DigitalInput *topSensor;
    frc::DigitalInput *bottomSensor;
    units::meter_t radius;
    units::kilogram_t mass;
    units::meter_t maxHeight = 1.33_m;
    units::meter_t minHeight = 0.28_m;
    units::meter_t initialHeight = 0_m;
    PIDConfig<units::meter, units::volt> pid;

    void WriteNT(std::shared_ptr<nt::NetworkTable> table);
  };

  //allows the states to be useable
  class Elevator : public behaviour::HasBehaviour {
   public: 
    Elevator(ElevatorConfig params);

    //creates functions for the states with the nessesary information
    void OnUpdate(units::second_t dt);

    void SetManual(units::volt_t voltage);
    void SetPID(units::meter_t height);
    void SetIdle();
    void SetRaw(units::volt_t voltage);

    ElevatorConfig &GetConfig();
    
    bool IsStable() const;
    ElevatorState GetState() const;

    units::meter_t GetHeight() const;
    units::meters_per_second_t MaxSpeed() const;
  
   private:
   //information that cannot be changed or edited by user
    units::volt_t _setpointManual{0};

    ElevatorConfig _config;
    ElevatorState _state;

    PIDController<units::meter, units::volt> _pid;

    std::shared_ptr<nt::NetworkTable> _table;

    units::volt_t _voltage{0};
  };
};