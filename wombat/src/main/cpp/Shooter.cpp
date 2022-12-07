#include "Shooter.h"

#include <networktables/NetworkTableInstance.h>

using namespace wom;

Shooter::Shooter(ShooterParams params) 
  : _params(params), _state(ShooterState::kIdle), 
    _pid{params.pid}, 
    _table(nt::NetworkTableInstance::GetDefault().GetTable("shooter")) {}

void Shooter::OnUpdate(units::second_t dt) {
  units::volt_t voltage{0};
  units::revolutions_per_minute_t currentSpeed = _params.gearbox.encoder->GetEncoderAngularVelocity() / _params.gearbox.reduction;

  switch(_state) {
    case ShooterState::kManual:
      voltage = _setpointManual;
      break;
    case ShooterState::kPID:
      {
        auto feedforward = _params.gearbox.motor.Voltage(0_Nm, _pid.setpoint);
        voltage = _pid.Calculate(currentSpeed, dt, feedforward);
      }
      break;
    case ShooterState::kIdle:
      voltage = 0_V;
      break;
  }

  units::newton_meter_t max_torque_at_current_limit = _params.gearbox.motor.Torque(_params.currentLimit);
  units::volt_t max_voltage_for_current_limit = _params.gearbox.motor.Voltage(max_torque_at_current_limit, currentSpeed);

  voltage = 1_V * std::min(voltage.value(), max_voltage_for_current_limit.value());

  _params.gearbox.transmission->SetVoltage(voltage);

  _table->GetEntry("output_volts").SetDouble(voltage.value());
  _table->GetEntry("speed_rpm").SetDouble(currentSpeed.value());
  _table->GetEntry("setpoint_rpm").SetDouble(units::revolutions_per_minute_t{_pid.setpoint}.value());
}

void Shooter::SetManual(units::volt_t voltage) {
  _state = ShooterState::kManual;
  _setpointManual = voltage;

}

void Shooter::SetPID(units::radians_per_second_t goal) {
  _state = ShooterState::kPID;
  _pid.setpoint = goal;
}

void Shooter::SetIdle() {
  _state = ShooterState::kIdle;
}

bool Shooter::IsStable() const {
  return _pid.IsStable();
}