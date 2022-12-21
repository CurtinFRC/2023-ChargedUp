#include "Encoder.h"

using namespace wom;


int Encoder::GetEncoderTicks() {
  return GetEncoderRawTicks() - _offset;
}

int Encoder::GetEncoderTicksPerRotation() {
  return _encoderTicksPerRotation;
}

void Encoder::ZeroEncoder() {
  _offset = GetEncoderRawTicks();
}

units::radian_t Encoder::GetEncoderPosition() {
  return (GetEncoderTicks() / (double)GetEncoderTicksPerRotation()) * 1_rad;
}

units::radians_per_second_t Encoder::GetEncoderAngularVelocity() {
  // return GetEncoderTickVelocity() / (double)GetEncoderTicksPerRotation() * 2 * 3.1415926;
  units::turns_per_second_t n_turns_per_s{GetEncoderTickVelocity() / GetEncoderTicksPerRotation()};
  return n_turns_per_s;
}

int DigitalEncoder::GetEncoderRawTicks() {
  return _nativeEncoder.Get();
}

double DigitalEncoder::GetEncoderTickVelocity() {
  return 1.0 / (double)_nativeEncoder.GetPeriod();
}

int DigitalEncoder::GetChannelA() {
  return _channelA;
}

int DigitalEncoder::GetChannelB() {
  return _channelB;
}

int DigitalEncoder::GetSimulationHandle() {
  return _nativeEncoder.GetFPGAIndex();
}

CANSparkMaxEncoder::CANSparkMaxEncoder(rev::CANSparkMax *controller)
  : Encoder(42), _encoder(controller->GetEncoder()) {}

int CANSparkMaxEncoder::GetEncoderRawTicks() {
  return _encoder.GetPosition() * GetEncoderTicksPerRotation();
}

double CANSparkMaxEncoder::GetEncoderTickVelocity() {
  return _encoder.GetVelocity() * GetEncoderTicksPerRotation() / 60;
}