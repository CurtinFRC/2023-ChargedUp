#include "Elevator.h"
#include <networktables/NetworkTableInstance.h>
#include <iostream>

#include <units/acceleration.h>

using namespace wom;

Elevator::Elevator(ElevatorConfig config)
  : _config(config), _state(ElevatorState::kIdle),
  _pid{config.path + "/pid", config.pid},
  _table(nt::NetworkTableInstance::GetDefault().GetTable("elevator")) {}


void Elevator::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};

  units::meter_t height = _config.gearbox.encoder->GetEncoderPosition().value() * _config.radius;

  switch(_state) {
    case ElevatorState::kIdle:
      voltage = 0_V;
      break;
    case ElevatorState::kManual:
      voltage = _setpointManual;
      break;
    case ElevatorState::kPID:
      {
        auto feedforward = _config.gearbox.motor.Voltage((_config.mass * 9.81_mps_sq) * _config.radius, 0_rad_per_s);
        voltage = _pid.Calculate(height, dt, feedforward);
      }
      break;
    case ElevatorState::kZeroing:
      if (!_config.bottomSensor) {
        std::cout << "No bottom sensor found, can't zero" << std::endl;
        _state = ElevatorState::kIdle;
      } else {
        voltage = -3_V;
        if (_config.bottomSensor->Get()) {
          _config.gearbox.encoder->ZeroEncoder();
          _state = ElevatorState::kIdle;
        }
      }
      break;
  }

  if (_config.bottomSensor && voltage < 0_V && _config.bottomSensor->Get()) {
    voltage = 0_V;
  } 
  if (_config.topSensor && voltage > 0_V && _config.topSensor->Get()) {
    voltage = 0_V;
  }
  _config.gearbox.transmission->SetVoltage(voltage);
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
wom::sim::ElevatorSim::ElevatorSim(ElevatorConfig config)
  : config(config),
    sim(config.gearbox.motor, 1.0, config.mass, config.radius,
        0_m, config.maxHeight, true),
    encoder(config.gearbox.encoder->MakeSimEncoder()),
    lowerLimit(config.bottomSensor ? new frc::sim::DIOSim(*config.bottomSensor) : nullptr),
    upperLimit(config.topSensor ? new frc::sim::DIOSim(*config.topSensor) : nullptr),
    table(nt::NetworkTableInstance::GetDefault().GetTable(config.path + "/sim"))
  {}

void wom::sim::ElevatorSim::Update(units::second_t dt) {
  sim.SetInputVoltage(config.gearbox.transmission->GetEstimatedRealVoltage());
  sim.Update(dt);

  encoder->SetEncoderTurns(1_rad * sim.GetPosition() / config.radius);
  if (lowerLimit) lowerLimit->SetValue(sim.HasHitLowerLimit());
  if (upperLimit) upperLimit->SetValue(sim.HasHitUpperLimit());

  table->GetEntry("height").SetDouble(sim.GetPosition().value());
  table->GetEntry("current").SetDouble(sim.GetCurrentDraw().value());
  table->GetEntry("velocity").SetDouble(sim.GetVelocity().value());
}

units::meter_t wom::sim::ElevatorSim::GetHeight() const {
  return sim.GetPosition();
}

units::ampere_t wom::sim::ElevatorSim::GetCurrent() const {
  return sim.GetCurrentDraw();
}