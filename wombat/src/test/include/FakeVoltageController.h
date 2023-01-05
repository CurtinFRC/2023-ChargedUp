#pragma once

#include "VoltageController.h"

namespace wom {
  class FakeVoltageController : public VoltageController {
   public:
    void SetVoltage(units::volt_t volt) override {
      _voltage = units::math::min(units::math::max(-_batVoltage, volt), _batVoltage);
    }

    units::volt_t GetVoltage() const override {
      return _voltage;
    }

    void SetBatteryVoltage(units::volt_t vb) {
      _batVoltage = vb;
    }

    void SetInverted(bool invert) override {
      _inverted = invert;
    }

    bool GetInverted() const override {
      return _inverted;
    }

   private:
    units::volt_t _voltage{0};
    units::volt_t _batVoltage{12};
    bool _inverted{false};
  };
}