#pragma once

#include <frc/I2C.h>
#include <frc/Notifier.h>
#include <units/length.h>
#include <optional>

class TOF {
 public:
  TOF (frc::I2C::Port port);

  std::optional<units::meter_t> GetDistance();

  void UpdateNow();
 private:
  frc::I2C _I2C;
  std::optional<units::meter_t> _currentValue;
  frc::Notifier _notifier;
};