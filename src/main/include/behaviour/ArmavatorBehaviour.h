#pragma once


#include "behaviour/Behaviour.h"
#include "Armavator.h"
#include "Grid.h"
#include <units/angle.h>
#include <units/length.h>

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
    Armavator *armavator;
    ArmavatorPosition setpoint;
    std::deque<grid_t::GridPathNode<units::second>> waypoints;
};
