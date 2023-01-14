#pragma once


#include "behaviour/Behaviour.h"
#include "Armavator.h"
#include "Grid.h"
#include <units/angle.h>
#include <units/length.h>

class ArmavatorGoToPositionBehaviour : public behaviour::Behaviour {
 public:
    using grid_t = wom::DiscretisedOccupancyGrid<units::radian, units::meter>;

    ArmavatorGoToPositionBehaviour(Armavator *armavator, grid_t grid, ArmavatorPosition setpoint);

    void OnStart() override;
    void OnTick(units::second_t dt) override;

 private:
    Armavator *armavator;
    grid_t grid;
    ArmavatorPosition setpoint;
    std::deque<grid_t::Idx_t> waypoints;
};
