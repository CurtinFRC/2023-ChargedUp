#pragma once

#include <frc/interfaces/Gyro.h>

namespace wom {
  class NavX : public frc::Gyro {
   public:
    NavX();
    ~NavX();
    /* From frc::Gyro */
    void Calibrate() override;
    void Reset() override;
    double GetAngle() const override;
    double GetRate() const override;

    void SetAngle(units::radian_t angle);
   private:
    class Impl;
    Impl *impl;
  };
}