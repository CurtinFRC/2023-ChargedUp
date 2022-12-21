#pragma once

#include "VoltageController.h"

namespace wom {
  class FakeVoltageController : public VoltageController {
   public:
    void SetVoltage(units::volt_t volt) override {
      _voltage = volt;
    }

    units::volt_t GetVoltage() const override {
      return _voltage;
    }

    void SetInverted(bool invert) override {
      _inverted = invert;
    }

    bool GetInverted() const override {
      return _inverted;
    }

   private:
    units::volt_t _voltage{0};
    bool _inverted{false};
  };
}