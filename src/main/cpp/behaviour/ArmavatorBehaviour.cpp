#include "behaviour/ArmavatorBehaviour.h"


//Constructs class
ArmavatorGoToPositionBehaviour::ArmavatorGoToPositionBehaviour(Armavator *armavator, ArmavatorPosition setpoint)
: armavator(armavator), setpoint(setpoint) {
    //tells code that the points are controlled (one point at a time) 
    Controls(armavator);
}


//Function for OnStart
void ArmavatorGoToPositionBehaviour::OnStart() {
    //Sets current position
    ArmavatorPosition current = armavator->GetCurrentPosition();
    //Sets positions information for the start and the end of the instructions
    grid_t::Idx_t start = armavator->config.grid.Discretise({current.angle, current.height});
    grid_t::Idx_t end = armavator->config.grid.Discretise({setpoint.angle, setpoint.height});
    //Sets arm and elevator speeds for start and end
    waypoints = armavator->config.grid.AStar<units::second>(
        start, end,
        1 / (armavator->arm.MaxSpeed() * 0.8),
        1 / (armavator->elevator.MaxSpeed() * 0.8)
    );
}

//Function for OnTick
void ArmavatorGoToPositionBehaviour::OnTick(units::second_t dt) {
    //If statement for targetted waypoint position is empty
    if (!waypoints.empty()) {
        grid_t::GridPathNode<units::second> waypoint = waypoints.front();
        while (!waypoints.empty() && waypoint.cost <= GetRunTime()) {
            waypoints.pop_front();
            if (!waypoints.empty())
                waypoint = waypoints.front();
        }

        armavator->SetPosition({waypoint.position.y, waypoint.position.x});
    
    //If waypoint is full, set next position
    } else {
        armavator->SetPosition(setpoint);

        //If the arm elevator is in correct final position, stop moving
        if (armavator->IsStable())
            SetDone();
    }
    
}