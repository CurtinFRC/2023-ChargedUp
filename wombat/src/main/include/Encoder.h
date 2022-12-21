#pragma once

#include <frc/Encoder.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include <rev/CANSparkMax.h>

#include "Util.h"

namespace wom {
  class Encoder {
   public:
    Encoder(double encoderTicksPerRotation) : _encoderTicksPerRotation(encoderTicksPerRotation){};
    virtual double    GetEncoderRawTicks() const = 0;
    virtual double    GetEncoderTickVelocity() const = 0;  // ticks/s
    virtual void      ZeroEncoder();

    double  GetEncoderTicks() const;
    double  GetEncoderTicksPerRotation() const;

    units::radian_t GetEncoderPosition();
    units::radians_per_second_t GetEncoderAngularVelocity();   // rad/s

   private:
    double _encoderTicksPerRotation;
    double _offset = 0;
  };

  class DigitalEncoder : public Encoder {
   public:
    DigitalEncoder(int channelA, int channelB, uint64_t ticksPerRotation)
        : Encoder(ticksPerRotation),
          _nativeEncoder(channelA, channelB){};

    double GetEncoderRawTicks() const override;
    double GetEncoderTickVelocity() const override;

   private:
    frc::Encoder _nativeEncoder;
  };

  class CANSparkMaxEncoder : public Encoder {
   public:
    CANSparkMaxEncoder(rev::CANSparkMax *controller);

    double GetEncoderRawTicks() const override;
    double GetEncoderTickVelocity() const override;
   private:
    rev::SparkMaxRelativeEncoder _encoder;
  };
}  // namespace wom
