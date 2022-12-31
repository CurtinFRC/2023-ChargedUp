#pragma once

#include "Encoder.h"

namespace wom {
  class FakeEncoder : public Encoder {
   public:
    FakeEncoder(double ticksPerRev) : Encoder(ticksPerRev) {}

    double GetEncoderRawTicks() const override {
      return _rotations.value() * GetEncoderTicksPerRotation();
    }

    double GetEncoderTickVelocity() const override {
      return _speed.value() * GetEncoderTicksPerRotation();
    }

    void SetTurns(units::turn_t turns) {
      _rotations = turns;
    }

    void SetTurnVelocity(units::turns_per_second_t speed, units::second_t dt) {
      _rotations += speed * dt;
      _speed = speed;
    }
   private:
    units::turn_t _rotations;
    units::turns_per_second_t _speed;
  };
}