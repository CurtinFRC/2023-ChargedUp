#include "Encoder.h"
#include <iostream>

using namespace wom;

double Encoder::GetEncoderTicks() const {
  return GetEncoderRawTicks();
}

double Encoder::GetEncoderTicksPerRotation() const {
  return _encoderTicksPerRotation * _reduction;
}

void Encoder::ZeroEncoder() {
  // _offset = GetEncoderRawTicks();
}

void Encoder::SetEncoderPosition(units::radian_t position) {
  // units::turn_t offset_turns = position - GetEncoderPosition();
  // _offset = -offset_turns.value() * GetEncoderTicksPerRotation();
}

void Encoder::SetEncoderOffset(units::radian_t offset) {
  _offset = offset;
  // units::turn_t offset_turns = offset;
  // _offset = offset_turns.value() * GetEncoderTicksPerRotation();
}

void Encoder::SetReduction(double reduction) {
  _reduction = reduction;
}

double Encoder::GetAbsoluteEncoderPosition() {
  return GetAbsoluteEncoderPosition();
}

// units::radian_t Encoder::GetEncoderPosition() {
//   units::turn_t n_turns{GetEncoderTicks() / GetEncoderTicksPerRotation()};
//   return n_turns;
// }

units::radian_t Encoder::GetEncoderPosition() {
  if (_type == 0) {
    units::turn_t n_turns{GetEncoderTicks() / GetEncoderTicksPerRotation()};
    return n_turns;
  } else {
    units::degree_t pos = (GetEncoderTicks()) * 1_deg;
    return pos  - _offset;
  }
}

units::radians_per_second_t Encoder::GetEncoderAngularVelocity() {
  // return GetEncoderTickVelocity() / (double)GetEncoderTicksPerRotation() * 2 * 3.1415926;
  units::turns_per_second_t n_turns_per_s{GetEncoderTickVelocity() / GetEncoderTicksPerRotation()};
  return n_turns_per_s;
}

double DigitalEncoder::GetEncoderRawTicks() const {
  // Encoder.encoderType = 0;
  return _nativeEncoder.Get();
}

double DigitalEncoder::GetEncoderTickVelocity() const {
  // return 1.0 / (double)_nativeEncoder.GetPeriod();
  return _nativeEncoder.GetRate();
}

// double DigitalEncoder::GetAbsoluteEncoderPosition() const {
//   return 0;
// }

CANSparkMaxEncoder::CANSparkMaxEncoder(rev::CANSparkMax *controller, double reduction)
  : Encoder(42, reduction, 0), _encoder(controller->GetEncoder()) {}

double CANSparkMaxEncoder::GetEncoderRawTicks() const {
  // Encoder.encoderType = 0;
  #ifdef PLATFORM_ROBORIO
    return _encoder.GetPosition() * GetEncoderTicksPerRotation();
  #else
    return _simTicks;
  #endif
}

double CANSparkMaxEncoder::GetEncoderTickVelocity() const {
  #ifdef PLATFORM_ROBORIO
    return _encoder.GetVelocity() * GetEncoderTicksPerRotation() / 60;
  #else
    return _simVelocity;
  #endif
}

// double CANSparkMaxEncoder::GetAbsoluteEncoderPosition() const {
//   return 0;
// }

TalonFXEncoder::TalonFXEncoder(ctre::phoenix::motorcontrol::can::TalonFX *controller, double reduction)
  : Encoder(2048, reduction, 0), _controller(controller) {
    controller->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::TalonFXFeedbackDevice::IntegratedSensor);
  }

double TalonFXEncoder::GetEncoderRawTicks() const {
  // Encoder.encoderType = 0;
  return _controller->GetSelectedSensorPosition();
}

double TalonFXEncoder::GetEncoderTickVelocity() const {
  return _controller->GetSelectedSensorVelocity() * 10;
}

// double TalonFXEncoder::GetAbsoluteEncoderPosition() const {
//   return 0;
// }

TalonSRXEncoder::TalonSRXEncoder(ctre::phoenix::motorcontrol::can::TalonSRX *controller, double ticksPerRotation, double reduction) 
  : Encoder(ticksPerRotation, reduction, 0), _controller(controller) {
    controller->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::TalonSRXFeedbackDevice::QuadEncoder);
  }

double TalonSRXEncoder::GetEncoderRawTicks() const {
  // Encoder.encoderType = 0;
  return _controller->GetSelectedSensorPosition();
}

double TalonSRXEncoder::GetEncoderTickVelocity() const {
  return _controller->GetSelectedSensorVelocity() * 10;
}

// double TalonSRXEncoder::GetAbsoluteEncoderPosition() const {
//   return 0;
// }

DutyCycleEncoder::DutyCycleEncoder(int channel, double ticksPerRotation, double reduction) 
  : Encoder(ticksPerRotation, reduction, 0), _dutyCycleEncoder(channel) {}

double DutyCycleEncoder::GetEncoderRawTicks() const {
  // Encoder.encoderType = 0;
  return _dutyCycleEncoder.Get().value();
}

double DutyCycleEncoder::GetEncoderTickVelocity() const {
  return 0;
}

// double DutyCycleEncoder::GetAbsoluteEncoderPosition() const {
//   return 0;
// }

CanEncoder::CanEncoder(int deviceNumber, double ticksPerRotation, double reduction)
  : Encoder(ticksPerRotation, reduction, 1) {
    _canEncoder = new CANCoder(deviceNumber);
  }

double CanEncoder::GetEncoderRawTicks() const {
  // std::cout << _canEncoder->GetAbsolutePosition() << std::endl;
  return _canEncoder->GetAbsolutePosition();
  // return const_cast<_canEncoder *>(this)->GetPosition();
  // return 100;
  // return _canEncoder.GetPosition() ? ;
  // return const_cast<_canEncoder*>(this)->GetPosition();
}

double CanEncoder::GetEncoderTickVelocity() const {
  // return 0;
  return _canEncoder->GetVelocity();
}

// double CanEncoder::GetAbsoluteEncoderPosition() {
//   return _canEncoder->GetAbsolutePosition();
// }

/* SIM */
#include "frc/simulation/EncoderSim.h"

class SimDigitalEncoder : public sim::SimCapableEncoder {
 public:
  SimDigitalEncoder(wom::Encoder *encoder, frc::Encoder *frcEncoder) : encoder(encoder), sim(*frcEncoder) {}
  
  void SetEncoderTurns(units::turn_t turns) override {
    sim.SetCount(turns.value() * encoder->GetEncoderTicksPerRotation());
  }

  void SetEncoderTurnVelocity(units::turns_per_second_t speed) override {
    sim.SetRate(speed.value() * encoder->GetEncoderTicksPerRotation());
  }
 private:
  wom::Encoder *encoder;
  frc::sim::EncoderSim sim;
};

std::shared_ptr<sim::SimCapableEncoder> DigitalEncoder::MakeSimEncoder() {
  return std::make_shared<SimDigitalEncoder>(this, &_nativeEncoder);
}

namespace wom {
  class SimCANSparkMaxEncoder : public sim::SimCapableEncoder {
  public:
    SimCANSparkMaxEncoder(wom::CANSparkMaxEncoder *encoder) : encoder(encoder) {}

    void SetEncoderTurns(units::turn_t turns) override {
      encoder->_simTicks = turns.value() * encoder->GetEncoderTicksPerRotation();
    }

    void SetEncoderTurnVelocity(units::turns_per_second_t speed) override {
      encoder->_simVelocity = speed.value() * encoder->GetEncoderTicksPerRotation();
    }
  private:
    wom::CANSparkMaxEncoder *encoder;
  };
}

std::shared_ptr<sim::SimCapableEncoder> CANSparkMaxEncoder::MakeSimEncoder() {
  return std::make_shared<SimCANSparkMaxEncoder>(this);
}

class SimTalonFXEncoder : public sim::SimCapableEncoder {
 public:
  SimTalonFXEncoder(wom::Encoder *encoder, TalonFX *talonFX) : encoder(encoder), sim(talonFX->GetSimCollection()) {}

  void SetEncoderTurns(units::turn_t turns) override {
    sim.SetIntegratedSensorRawPosition(turns.value() * encoder->GetEncoderTicksPerRotation());
  }

  void SetEncoderTurnVelocity(units::turns_per_second_t speed) override {
    sim.SetIntegratedSensorVelocity(speed.value() * encoder->GetEncoderTicksPerRotation() / 10.0);
  }
 private:
  wom::Encoder *encoder;
  ctre::phoenix::motorcontrol::TalonFXSimCollection &sim;
};

std::shared_ptr<sim::SimCapableEncoder> TalonFXEncoder::MakeSimEncoder() {
  return std::make_shared<SimTalonFXEncoder>(this, _controller);
}

std::shared_ptr<sim::SimCapableEncoder> TalonSRXEncoder::MakeSimEncoder() {
  return nullptr;
}

std::shared_ptr<sim::SimCapableEncoder> DutyCycleEncoder::MakeSimEncoder() {
  return nullptr;
}

std::shared_ptr<sim::SimCapableEncoder> CanEncoder::MakeSimEncoder() {
  return nullptr;
}


