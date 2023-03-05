#pragma once

#include <frc/XboxController.h>
#include "behaviour/Behaviour.h"
#include "Armavator.h"
#include "Grid.h"
#include <units/angle.h>
#include <units/length.h>
#include <iostream>
#include <frc/event/EventLoop.h>
#include "ControlUtil.h"

enum class ArmavatorAutoSetpointEnum {
  kInIntake,
  kTravel,
  kFrontMidPlace,
  kFrontLowPlace, 
  kBackHighPlace, 
  kBackMidPlace, 
  kBackLowPlace,
  kWaitToCollect
};

class ArmavatorGoToAutoSetpoint : public behaviour::Behaviour {
 public: 
  ArmavatorGoToAutoSetpoint(Armavator *armavator, units::meter_t height, units::degree_t angle);

  void OnStart();
  void OnTick(units::second_t dt) override;
 private: 
  Armavator *_armavator;

  units::degree_t _angle;
  units::meter_t _height;
  // ArmavatorAutoSetpointEnum _setpoint;

  // ArmavatorPosition _setpointValue;
};

class ArmavatorGoToPositionBehaviour : public behaviour::Behaviour {
 public:
   using grid_t = ArmavatorConfig::grid_t;

   //constructor for class
   ArmavatorGoToPositionBehaviour(Armavator *armavator, ArmavatorPosition setpoint);

   //Override the OnStart abd OnTick functions, while setting the units for when Armavator runs
   void OnStart() override;
   void OnTick(units::second_t dt) override;

 private:
   //stores nessesary information that can't be changed
   Armavator *_armavator;

   ArmavatorPosition _setpoint;
   std::deque<grid_t::GridPathNode<units::second>> _waypoints;
};

// class ArmavatorManualBehaviour : public behaviour::Behaviour {
//  public:
//   using grid_t = ArmavatorConfig::grid_t;

//   ArmavatorManualBehaviour(Armavator *armavator, frc::XboxController &codriver);

//   void OnStart() override;
//   void OnTick(units::second_t dt) override;
//  private:
//   Armavator *_armavator;

//   ArmavatorPosition _setpoint;
//   std::deque<grid_t::GridPathNode<units::second>> _waypoints;
//   frc::XboxController &_codriver;
// };

class ArmavatorRawBehaviour : public behaviour::Behaviour {
 public:
  using grid_t = ArmavatorConfig::grid_t;

  //constructor
  ArmavatorRawBehaviour(Armavator *armavator, frc::XboxController &codriver);

  void OnStart() override;
  void OnTick(units::second_t dt) override;
 private:
  Armavator *_armavator;

  ArmavatorPosition _setpoint;
  std::deque<grid_t::GridPathNode<units::second>> _waypoints;
  frc::XboxController &_codriver;
};


class ArmavatorManualBehaviour : public behaviour::Behaviour {
 public: 
  ArmavatorManualBehaviour(Armavator *armavator, frc::XboxController &codriver);

  void OnStart() override;
  void OnTick(units::second_t dt) override;
 private: 
  Armavator *_armavator;
  ArmavatorPosition _manualSetpoint;
  ArmavatorPosition _setpointValue;

  frc::XboxController &_codriver;

  units::meter_t startHeight; 
  frc::EventLoop *loop;

  bool rawControl = true;
};