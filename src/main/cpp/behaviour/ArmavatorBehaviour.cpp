#include "behaviour/ArmavatorBehaviour.h"

#include <iostream>

ArmavatorGoToPositionBehaviour::ArmavatorGoToPositionBehaviour(Armavator *armavator, ArmavatorPosition setpoint)
: armavator(armavator), setpoint(setpoint) {
    Controls(armavator);
}

void ArmavatorGoToPositionBehaviour::OnStart() {
    ArmavatorPosition current = armavator->GetCurrentPosition();
    grid_t::Idx_t start = armavator->config.grid.Discretise({current.angle, current.height});
    grid_t::Idx_t end = armavator->config.grid.Discretise({setpoint.angle, setpoint.height});
    waypoints = armavator->config.grid.AStar<units::second>(
        start, end,
        1 / armavator->arm.MaxSpeed(),
        1 / armavator->elevator.MaxSpeed()
    );

    std::cout << armavator->config.grid._grid << std::endl;

    for (auto wp : waypoints) {
        std::cout << wp.transpose() << std::endl;
    }
}

void ArmavatorGoToPositionBehaviour::OnTick(units::second_t dt) {
    if (!waypoints.empty()) {
        grid_t::Idx_t waypoint = waypoints.front();
        ArmavatorPosition currentPosition = armavator->GetCurrentPosition();
        grid_t::Idx_t current = armavator->config.grid.Discretise({currentPosition.angle, currentPosition.height});
        
        grid_t::ContinuousIdxT target = armavator->config.grid.CenterOf(waypoint);
        armavator->SetPosition({target.y, target.x});

        if (waypoint == current) {
            waypoints.pop_front();
        }
    } else {
        armavator->SetPosition(setpoint);

        if (armavator->IsStable())
            SetDone();
    }
    
}