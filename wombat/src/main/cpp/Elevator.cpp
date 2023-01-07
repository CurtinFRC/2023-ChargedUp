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

/* SIMULATION */
wom::sim::ElevatorSim::ElevatorSim(ElevatorParams params)
  : params(params),
    sim(params.gearbox.motor, 1.0, params.mass, params.radius,
        0_m, params.maxHeight, true),
    encoder(params.gearbox.encoder->MakeSimEncoder()),
    lowerLimit(params.bottomSensor ? new frc::sim::DIOSim(*params.bottomSensor) : nullptr),
    upperLimit(params.topSensor ? new frc::sim::DIOSim(*params.topSensor) : nullptr)
  {}

void wom::sim::ElevatorSim::Update(units::volt_t voltage, units::second_t dt) {
  sim.SetInputVoltage(voltage);
  sim.Update(dt);

  encoder->SetEncoderTurns(1_rad * sim.GetPosition() / params.radius);
  if (lowerLimit) lowerLimit->SetValue(sim.HasHitLowerLimit());
  if (upperLimit) upperLimit->SetValue(sim.HasHitUpperLimit());

  nt::NetworkTableInstance::GetDefault().GetEntry("elevator/sim/height").SetDouble(sim.GetPosition().value());
}

units::meter_t wom::sim::ElevatorSim::GetHeight() const {
  return sim.GetPosition();
}