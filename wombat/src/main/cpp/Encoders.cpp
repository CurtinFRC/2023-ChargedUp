#include "Encoder.h"

using namespace wom;

double Encoder::GetEncoderTicks() const {
  return GetEncoderRawTicks() - _offset;
}

double Encoder::GetEncoderTicksPerRotation() const {
  return _encoderTicksPerRotation;
}

void Encoder::ZeroEncoder() {
  _offset = GetEncoderRawTicks();
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
  return 1.0 / (double)_nativeEncoder.GetPeriod();
}

CANSparkMaxEncoder::CANSparkMaxEncoder(rev::CANSparkMax *controller)
  : Encoder(42), _encoder(controller->GetEncoder()) {}

double CANSparkMaxEncoder::GetEncoderRawTicks() const {
  return _encoder.GetPosition() * GetEncoderTicksPerRotation();
}

double CANSparkMaxEncoder::GetEncoderTickVelocity() const {
  return _encoder.GetVelocity() * GetEncoderTicksPerRotation() / 60;
}