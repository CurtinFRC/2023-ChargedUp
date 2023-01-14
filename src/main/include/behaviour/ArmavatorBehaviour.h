#pragma once


#include "behaviour/Behaviour.h"
#include "Armavator.h"
#include "Grid.h"
#include <units/angle.h>
#include <units/length.h>

class ArmavatorGoToPositionBehaviour : public behaviour::Behaviour {
 public:
    using grid_t = ArmavatorConfig::grid_t;

    ArmavatorGoToPositionBehaviour(Armavator *armavator, ArmavatorPosition setpoint);

    void OnStart() override;
    void OnTick(units::second_t dt) override;

 private:
    Armavator *armavator;
    ArmavatorPosition setpoint;
    std::deque<grid_t::GridPathNode<units::second>> waypoints;
};
