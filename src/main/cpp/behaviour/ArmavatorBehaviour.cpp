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
        1 / (armavator->arm.MaxSpeed() * 0.8),
        1 / (armavator->elevator.MaxSpeed() * 0.8)
    );
}

void ArmavatorGoToPositionBehaviour::OnTick(units::second_t dt) {
    if (!waypoints.empty()) {
        grid_t::GridPathNode<units::second> waypoint = waypoints.front();
        while (!waypoints.empty() && waypoint.cost <= GetRunTime()) {
            waypoints.pop_front();
            if (!waypoints.empty())
                waypoint = waypoints.front();
        }

        armavator->SetPosition({waypoint.position.y, waypoint.position.x});
    } else {
        armavator->SetPosition(setpoint);

        if (armavator->IsStable())
            SetDone();
    }
    
}