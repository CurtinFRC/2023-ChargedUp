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
    case ArmState::kZeroing:
      voltage = -2_V;
      if (_config.lowerLimitSwitch->Get()) {
        _config.gearbox.encoder->ZeroEncoder();
        _state = ArmState::kIdle;
        /*when bottom limit switch is triggered, the encoder is zeroed and returns to the idle state*/
      }
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

void Arm::SetZeroing() {
  _state = ArmState::kZeroing;
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

bool Arm::IsStable() const {
  return _pid.IsStable();
}

/* SIMULATION */
#include <units/math.h>

::wom::sim::ArmSim::ArmSim(ArmConfig config) 
  : config(config),
    angle(config.initialAngle),
    encoder(config.gearbox.encoder->MakeSimEncoder()),
    lowerLimit(config.lowerLimitSwitch ? new frc::sim::DIOSim(*config.lowerLimitSwitch) : nullptr),
    upperLimit(config.upperLimitSwitch ? new frc::sim::DIOSim(*config.upperLimitSwitch) : nullptr),
    table(nt::NetworkTableInstance::GetDefault().GetTable(config.path + "/sim"))
  {}

units::ampere_t wom::sim::ArmSim::GetCurrent() const {
  return current;
}

void ::wom::sim::ArmSim::Update(units::second_t dt) {
  torque = (config.loadMass * config.armLength + config.armMass * config.armLength / 2.0) * 9.81_m / 1_s / 1_s * units::math::cos(config.angleOffset + angle) + additionalTorque;
  velocity = config.gearbox.motor.Speed(torque, config.gearbox.transmission->GetEstimatedRealVoltage());
  angle += velocity * dt;

  if (angle <= config.minAngle) {
    angle = config.minAngle;
    velocity = 0_rad / 1_s;
    if (lowerLimit) lowerLimit->SetValue(true);
  } else {
    if (lowerLimit) lowerLimit->SetValue(false);
  }

  if (angle >= config.maxAngle) {
    angle = config.maxAngle;
    velocity = 0_rad / 1_s;
    if (upperLimit) upperLimit->SetValue(true);
  } else {
    if (upperLimit) upperLimit->SetValue(false);
  }

  current = config.gearbox.motor.Current(velocity, config.gearbox.transmission->GetEstimatedRealVoltage());

  if (encoder) encoder->SetEncoderTurns(angle - config.initialAngle);

  table->GetEntry("angle").SetDouble(angle.convert<units::degree>().value());
  table->GetEntry("current").SetDouble(current.value());
}
