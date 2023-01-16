#include "Arm.h"

#include <units/math.h>

using namespace frc;
using namespace wom;

void ArmConfig::WriteNT(std::shared_ptr<nt::NetworkTable> table) {
  table->GetEntry("armMass").SetDouble(armMass.value());
  table->GetEntry("loadMass").SetDouble(loadMass.value());
  table->GetEntry("armLength").SetDouble(armLength.value());
  table->GetEntry("minAngle").SetDouble(minAngle.convert<units::degree>().value());
  table->GetEntry("maxAngle").SetDouble(maxAngle.convert<units::degree>().value());
  table->GetEntry("initialAngle").SetDouble(initialAngle.convert<units::degree>().value());
  table->GetEntry("angleOffset").SetDouble(initialAngle.convert<units::degree>().value());
}

Arm::Arm(ArmConfig config)
  : _config(config),
    _pid(config.path + "/pid", config.pidConfig),
    _table(nt::NetworkTableInstance::GetDefault().GetTable(config.path))
{
  _config.gearbox.encoder->SetEncoderPosition(_config.initialAngle);
}

void Arm::OnUpdate(units::second_t dt) {
  units::volt_t voltage = 0_V;
  auto angle = GetAngle();

  switch (_state) {
    case ArmState::kIdle:
      break;
    case ArmState::kAngle:
      {
        units::newton_meter_t torque = 9.81_m / 1_s / 1_s * _config.armLength * units::math::cos(angle + _config.angleOffset) * (0.5 * _config.armMass + _config.loadMass);
        units::volt_t feedforward = _config.gearbox.motor.Voltage(torque, 0_rad/ 1_s);
        voltage = _pid.Calculate(angle, dt, feedforward);
      }
      break;
  }

  if (
    (((_config.minAngle + _config.angleOffset) < 75_deg && units::math::abs(_pid.GetSetpoint() - _config.minAngle) <= 1_deg)
     || ((_config.maxAngle + _config.angleOffset) > 105_deg && units::math::abs(_pid.GetSetpoint() - _config.maxAngle) <= 1_deg)) && 
    units::math::abs(_pid.GetError()) <= 1_deg
  ) {
    voltage = 0_V;
  }
  _config.gearbox.transmission->SetVoltage(voltage);

  _table->GetEntry("angle").SetDouble(angle.convert<units::degree>().value());
  _config.WriteNT(_table->GetSubTable("config"));
}

void Arm::SetIdle() {
  _state = ArmState::kIdle;
}

void Arm::SetAngle(units::radian_t angle) {
  _state = ArmState::kAngle;
  _pid.SetSetpoint(angle);
}

ArmConfig &Arm::GetConfig() {
  return _config;
}

units::radian_t Arm::GetAngle() const {
  return _config.gearbox.encoder->GetEncoderPosition();
}

units::radians_per_second_t Arm::MaxSpeed() const {
  return _config.gearbox.motor.Speed(0_Nm, 12_V);
}
