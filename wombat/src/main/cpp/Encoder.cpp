#include "Encoder.h"

using namespace wom;

double Encoder::GetEncoderTicks() const {
  return GetEncoderRawTicks() - _offset;
}

double Encoder::GetEncoderTicksPerRotation() const {
  return _encoderTicksPerRotation * _reduction;
}

void Encoder::ZeroEncoder() {
  _offset = GetEncoderRawTicks();
}

void Encoder::SetReduction(double reduction) {
  _reduction = reduction;
}

units::radian_t Encoder::GetEncoderPosition() {
  units::turn_t n_turns{GetEncoderTicks() / GetEncoderTicksPerRotation()};
  return n_turns;
}

units::radians_per_second_t Encoder::GetEncoderAngularVelocity() {
  // return GetEncoderTickVelocity() / (double)GetEncoderTicksPerRotation() * 2 * 3.1415926;
  units::turns_per_second_t n_turns_per_s{GetEncoderTickVelocity() / GetEncoderTicksPerRotation()};
  return n_turns_per_s;
}

double DigitalEncoder::GetEncoderRawTicks() const {
  return _nativeEncoder.Get();
}

double DigitalEncoder::GetEncoderTickVelocity() const {
  // return 1.0 / (double)_nativeEncoder.GetPeriod();
  return _nativeEncoder.GetRate();
}

CANSparkMaxEncoder::CANSparkMaxEncoder(rev::CANSparkMax *controller, double reduction)
  : Encoder(42, reduction), _encoder(controller->GetEncoder()) {}

double CANSparkMaxEncoder::GetEncoderRawTicks() const {
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

TalonFXEncoder::TalonFXEncoder(ctre::phoenix::motorcontrol::can::TalonFX *controller, double reduction)
  : Encoder(2048, reduction), _controller(controller) {
    controller->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::TalonFXFeedbackDevice::IntegratedSensor);
  }

double TalonFXEncoder::GetEncoderRawTicks() const {
  return _controller->GetSelectedSensorPosition();
}

double TalonFXEncoder::GetEncoderTickVelocity() const {
  return _controller->GetSelectedSensorVelocity() * 10;
}

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