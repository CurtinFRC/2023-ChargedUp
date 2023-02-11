#include "Elevator.h"
#include <networktables/NetworkTableInstance.h>
#include <iostream>

#include <units/acceleration.h>

using namespace wom;

void ElevatorConfig::WriteNT(std::shared_ptr<nt::NetworkTable> table) {
  table->GetEntry("radius").SetDouble(radius.value());
  table->GetEntry("mass").SetDouble(mass.value());
  table->GetEntry("maxHeight").SetDouble(maxHeight.value());
}

Elevator::Elevator(ElevatorConfig config)
  : _config(config), _state(ElevatorState::kIdle),
  _pid{config.path + "/pid", config.pid},
  _table(nt::NetworkTableInstance::GetDefault().GetTable(config.path)) {
  _config.gearbox.encoder->SetEncoderPosition(_config.initialHeight / _config.radius * 1_rad);
}


void Elevator::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};

  units::meter_t height = GetHeight();


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
  }

  // Top sensor detector
  if(_config.topSensor != nullptr) {
    if(_config.topSensor->Get()) {
      _config.gearbox.encoder->SetEncoderPosition(_config.maxHeight / _config.radius * 1_rad);
      //voltage = 0_V;
    }
  }

  //Bottom Sensor Detection
  if (_config.bottomSensor != nullptr) {
    if (_config.bottomSensor->Get()) {
      _config.gearbox.encoder->SetEncoderPosition(_config.minHeight / _config.radius * 1_rad);
      //voltage = 0_V;
    }
  }

  // Set voltage to motors...
  _config.gearbox.transmission->SetVoltage(voltage);
}

void Elevator::SetManual(units::volt_t voltage) {
  _state = ElevatorState::kManual;
  _setpointManual = voltage;
}

void Elevator::SetPID(units::meter_t height) {
  _state = ElevatorState::kPID;
  _pid.SetSetpoint(height);
}

void Elevator::SetIdle() {
  _state = ElevatorState::kIdle;
}

ElevatorConfig &Elevator::GetConfig() {
  return _config;
}

bool Elevator::IsStable() const {
  return _pid.IsStable();
}

ElevatorState Elevator::GetState() const {
  return _state;
}

units::meter_t Elevator::GetHeight() const {
  return _config.gearbox.encoder->GetEncoderPosition().value() * _config.radius;
}

units::meters_per_second_t Elevator::MaxSpeed() const {
  return _config.gearbox.motor.Speed((_config.mass * 9.81_mps_sq) * _config.radius, 12_V) / 1_rad * _config.radius;
}
