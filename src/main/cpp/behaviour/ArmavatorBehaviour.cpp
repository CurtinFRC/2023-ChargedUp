#include "behaviour/ArmavatorBehaviour.h"


//Constructs class
ArmavatorGoToPositionBehaviour::ArmavatorGoToPositionBehaviour(Armavator *armavator, ArmavatorPosition setpoint, frc::XboxController &codriver)
: _armavator(armavator), _setpoint(setpoint), _codriver(codriver) {
  //tells code that the points are controlled (one point at a time) 
  Controls(armavator);
};

// Function for OnStart
void ArmavatorGoToPositionBehaviour::OnStart() {
  std::cout << "On Start" << std::endl;
  // Zero the elevator
  // _armavator->elevator->SetZeroing();
  
  //Sets current position
  // ArmavatorPosition Elevator
  // ArmavatorPosition current = armavator->GetCurrentPosition();
  //Sets positions information for the start and the end of the instructions
  // grid_t::Idx_t start = armavator.config.grid.Discretise({current.angle, current.height});
  // grid_t::Idx_t end = armavator.config.grid.Discretise({setpoint.angle, setpoint.height});
  //Sets arm and elevator speeds for start and end
  // waypoints = armavator->config.grid.AStar<units::second>(
  //     start, end,
  //     1 / (armavator->arm.MaxSpeed() * 0.8),
  //     1 / (armavator->elevator.MaxSpeed() * 0.8)
  // );
}

//Function for OnTick
void ArmavatorGoToPositionBehaviour::OnTick(units::second_t dt) {
  std::cout << "Running" << std::endl;

  //If statement for targetted waypoint position is empty
  // if (!waypoints.empty()) {
  //     grid_t::GridPathNode<units::second> waypoint = waypoints.front();
  //     while (!waypoints.empty() && waypoint.cost <= GetRunTime()) {
  //         waypoints.pop_front();
  //         if (!waypoints.empty())
  //             waypoint = waypoints.front();
  //     }

  //     ArmavatorPosition currentPosition = armavator->GetCurrentPosition();
  //     grid_t::Idx_t current = armavator.config.grid.Discretise({currentPosition.angle, currentPosition.height});
      
  //     armavator->SetPosition({waypoint.position.y, waypoint.position.x});
  
  // //If waypoint is full, set next position
  // } else {
  //     armavator->SetPosition(setpoint);

  //     //If the arm elevator is in correct final position, stop moving
  if (_armavator->IsStable())
    SetDone();
  // }  
}

ArmavatorManualBehaviour::ArmavatorManualBehaviour(Armavator *armavator, frc::XboxController &codriver)
: _armavator(armavator), _codriver(codriver) {
  //tells code that the points are controlled (one point at a time) 
  _setpoint.height = 0.0_m;
  _setpoint.angle = 0.0_deg;
  Controls(armavator);
};

void ArmavatorManualBehaviour::OnStart() {
  std::cout << "On Start" << std::endl;

  // Zero the elevator
  _armavator->elevator->SetZeroing();
}

void ArmavatorManualBehaviour::OnTick(units::second_t dt) {
  //Raw Positioning
  if(!_codriver.GetRightX() && !_codriver.GetLeftX()) {
    _armavator->arm->GetConfig().gearbox.transmission->SetVoltage(0_V);
    _armavator->elevator->SetManual(0_V);
  } else{
    if(_codriver.GetRightX()) {
      _armavator->arm->GetConfig().gearbox.transmission->SetVoltage(_armavator->arm->GetRaw() * _codriver.GetRightX());
    } else if (_codriver.GetLeftX()) {
      _armavator->elevator->GetConfig().gearbox.transmission->SetVoltage(_armavator->elevator->GetRaw() * _codriver.GetLeftX());
    }
  }

  // Manual Positioning
  // Elevator
  if (std::abs(_codriver.GetRightY()) > 0.15) {
    _setpoint.height = _setpoint.height + (_codriver.GetRightY() * 1.0_m);
  }

  // Angle
  if(std::abs(_codriver.GetLeftY()) > 0.15) {
    _setpoint.angle = _setpoint.angle + (_codriver.GetLeftY() * 1.0_deg);
  }

  // Set Position
  _armavator->SetPosition(_setpoint);

  if(!_codriver.GetAButton() && !_codriver.GetBButton() && !_codriver.GetXButton() && !_codriver.GetYButton()) {
    units::volt_t voltage{0};
  } else {
    if(_codriver.GetAButton()) {
      _armavator->SetPosition({0.2_m, 0_deg});
    }
    if(_codriver.GetBButton()) {
      _armavator->SetPosition({1.2_m, 75_deg});
    }
    if(_codriver.GetXButton()) {
      _armavator->SetPosition({1.0_m, 240_deg});
    }
    if(_codriver.GetYButton()) {
      _armavator->SetPosition({0_m, 0_deg});
    }
  }
}