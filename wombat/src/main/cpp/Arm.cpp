#include "Arm.h"

using namespace frc;
using namespace wom;

Arm::Arm(ArmConfig config) : _config(config), _pid("arm/pid", config.pidConfig) {   
}

void Arm::OnUpdate(units::second_t dt) {
  units::volt_t voltage = 0_V;
  switch (_state) {
    case ArmState::kIdle:
      break;
    case ArmState::kZeroing:
      voltage = -2_V;
      if (_config.bottomLimitSwitch->Get()) {
        _config.gearbox.encoder->ZeroEncoder();
        _state = ArmState::kIdle;
        /*when bottom limit switch is triggered, the encoder is zeroed and returns to the idle state*/
      }
      break;
    case ArmState::kAngle:
      {
        voltage = _pid.Calculate(_config.gearbox.encoder->GetEncoderPosition(), dt);
        /*creates a pid controller*/
        // units::radian_t currentAngle = _config.gearbox.encoder->GetEncoderPosition();
        // units::radian_t error = _targetAngle - currentAngle;
        // voltage = 12_V / 20_deg * error;
      }
      break;
  }
  _config.gearbox.transmission->SetVoltage(voltage);
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
/* SIMULATION */
#include <units/math.h>

::wom::sim::ArmSim::ArmSim(wom::DCMotor motor, units::kilogram_t mass, units::meter_t armLength,
                    units::radian_t minAngle, units::radian_t maxAngle) 
  : motor(motor), nominalTorque(mass * 9.81_m / 1_s / 1_s * armLength),
    minAngle(minAngle), maxAngle(maxAngle) { }

void ::wom::sim::ArmSim::Update(units::volt_t voltage, units::second_t dt) {
  angle += motor.Speed(nominalTorque * units::math::cos(angle), voltage) * dt;

  if (angle <= minAngle) {
    angle = minAngle;
    lowerLimit = true;
  } else {
    lowerLimit = false;
  }

  if (angle >= maxAngle) {
    angle = maxAngle;
    upperLimit = true;
  } else {
    upperLimit = false;
  }

  nt::NetworkTableInstance::GetDefault().GetEntry("arm/sim/angle").SetDouble(angle.convert<units::degree>().value());
  nt::NetworkTableInstance::GetDefault().GetEntry("arm/sim/lowerLimit").SetBoolean(lowerLimit);
  nt::NetworkTableInstance::GetDefault().GetEntry("arm/sim/upperLimit").SetBoolean(upperLimit);
}

bool ::wom::sim::ArmSim::IsLowerLimit() const {
  return lowerLimit;
}

bool ::wom::sim::ArmSim::IsUpperLimit() const {
  return upperLimit;
}
