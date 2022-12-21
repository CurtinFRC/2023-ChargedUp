#pragma once 

#include <frc/interfaces/Gyro.h>

namespace wom {
  class FakeGyro : public frc::Gyro {
   public:
    void Calibrate() override {}
    void Reset() override {
      _angle = 0_deg;
    }

    double GetAngle() const override {
      return -_angle.value();
    }

    double GetRate() const override {
      return 0;
    }

    void SetAngle(units::degree_t angle) {
      _angle = angle;
    }
   private:
    units::degree_t _angle{0};
  };
}