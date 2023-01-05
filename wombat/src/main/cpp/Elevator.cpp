#include "Elevator.h"
#include <networktables/NetworkTableInstance.h>
#include <iostream>

#include <units/acceleration.h>

using namespace wom;

Elevator::Elevator(std::string path, ElevatorParams params)
  : _params(params), _state(ElevatorState::kIdle),
  _pid{path + "/pid", params.pid},
  _table(nt::NetworkTableInstance::GetDefault().GetTable("elevator")) {}


void Elevator::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};

  units::meter_t height = _params.gearbox.encoder->GetEncoderPosition().value() * _params.radius;

  switch(_state) {
    case ElevatorState::kIdle:
      voltage = 0_V;
      break;
    case ElevatorState::kManual:
      voltage = _setpointManual;
      break;
    case ElevatorState::kPID:
      {
        auto feedforward = _params.gearbox.motor.Voltage((_params.mass * 9.81_mps_sq) * _params.radius, 0_rad_per_s);
        voltage = _pid.Calculate(height, dt, feedforward);
      }
      break;
    case ElevatorState::kZeroing:
      if (!_params.bottomSensor) {
        std::cout << "No bottom sensor found, can't zero" << std::endl;
        _state = ElevatorState::kIdle;
      } else {
        voltage = -3_V;
        if (_params.bottomSensor->Get()) {
          _params.gearbox.encoder->ZeroEncoder();
          _state = ElevatorState::kIdle;
        }
      }
      break;
  }

  if (_params.bottomSensor && voltage < 0_V && _params.bottomSensor->Get()) {
    voltage = 0_V;
  } 
  if (_params.topSensor && voltage > 0_V && _params.topSensor->Get()) {
    voltage = 0_V;
  }
  _params.gearbox.transmission->SetVoltage(voltage);
}

void Elevator::SetManual(units::volt_t voltage) {
  _state = ElevatorState::kManual;
  _setpointManual = voltage;
}

void Elevator::SetPID() {
  _state = ElevatorState::kPID;
}

void Elevator::SetZeroing() {
  _state = ElevatorState::kZeroing;
}

void Elevator::SetIdle() {
  _state = ElevatorState::kIdle;
}