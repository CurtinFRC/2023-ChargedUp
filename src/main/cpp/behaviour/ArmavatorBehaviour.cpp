#include "behaviour/ArmavatorBehaviour.h"

ArmavatorGoToPositionBehaviour::ArmavatorGoToPositionBehaviour(Armavator *armavator, grid_t grid, ArmavatorPosition setpoint)
: armavator(armavator), grid(grid), setpoint(setpoint) { }

void ArmavatorGoToPositionBehaviour::OnStart() {
    ArmavatorPosition current = armavator->GetCurrentPosition();
    grid_t::Idx_t start = grid.Discretise({current.angle, current.height});
    grid_t::Idx_t end = grid.Discretise({setpoint.angle, setpoint.height});
    waypoints = grid.AStar<units::second>(
        start, end,
        1 / armavator->arm.MaxSpeed(),
        1 / armavator->elevator.MaxSpeed()
    );
}

void ArmavatorGoToPositionBehaviour::OnTick(units::second_t dt) {
    if (!waypoints.empty()) {
        grid_t::Idx_t waypoint = waypoints.front();
        ArmavatorPosition currentPosition = armavator->GetCurrentPosition();
        grid_t::Idx_t current = grid.Discretise({currentPosition.angle, currentPosition.height});
        
        grid_t::ContinuousIdxT target = grid.CenterOf(waypoint);
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