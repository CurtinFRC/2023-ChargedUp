#pragma once

#include "sim/SimGyro.h"
#include <frc/interfaces/Gyro.h>

namespace wom {
  class Gyro : public frc::Gyro {
   public:
    virtual std::shared_ptr<sim::SimCapableGyro> MakeSimGyro() = 0;
  };

  class NavX : public Gyro {
   public:
    NavX();
    ~NavX();
    /* From frc::Gyro */
    void Calibrate() override;
    void Reset() override;
    double GetAngle() const override;
    double GetRate() const override;

    units::radian_t GetPitch();
    units::radian_t GetRoll();

    void SetAngle(units::radian_t angle);

    std::shared_ptr<sim::SimCapableGyro> MakeSimGyro() override;
   private:
    class Impl;
    Impl *impl;
  };
}