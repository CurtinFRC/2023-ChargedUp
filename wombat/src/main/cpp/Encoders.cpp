#include "Encoder.h"

using namespace wom::sensors;

double Encoder::GetEncoderRotations() {
  return GetEncoderTicks() / (double)GetEncoderTicksPerRotation();
}

int Encoder::GetEncoderTicks() {
  return GetEncoderRawTicks() - _offset;
}

int Encoder::GetEncoderTicksPerRotation() {
  return _encoderTicksPerRotation;
}

void Encoder::ZeroEncoder() {
  _offset = GetEncoderRawTicks();
}

double Encoder::GetEncoderAngularVelocity() {
  return GetEncoderTickVelocity() / (double)GetEncoderTicksPerRotation() * 2 * 3.1415926;
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