#pragma once

#include <units/base.h>
#include <units/time.h>

#include <frc/filter/LinearFilter.h>

namespace wom {
  template<typename IN, typename OUT>
  struct PIDConfig {
    using kp_t = units::unit_t<units::compound_unit<OUT, units::inverse<IN>>>;
    using ki_t = units::unit_t<units::compound_unit<OUT, units::inverse<IN>, units::inverse<units::second>>>;
    using kd_t = units::unit_t<units::compound_unit<OUT, units::inverse<IN>, units::second>>;

    using error_t = units::unit_t<IN>;
    using deriv_t = units::unit_t<units::compound_unit<IN, units::inverse<units::second>>>;

    kp_t kp;
    ki_t ki{0};
    kd_t kd{0};

    error_t stableThresh{-1};
    deriv_t stableDerivThresh{100};
  };

  template<typename IN, typename OUT>
  class PIDController {
   public:
    using config_t = PIDConfig<IN, OUT>;
    using in_t = units::unit_t<IN>;
    using out_t = units::unit_t<OUT>;

    config_t config;
    in_t setpoint;

    PIDController(config_t initialGains, in_t setpoint = in_t{0}) 
      : config(initialGains), setpoint(setpoint),
        _posFilter(frc::LinearFilter<typename config_t::error_t>::MovingAverage(5)),
        _velFilter(frc::LinearFilter<typename config_t::deriv_t>::MovingAverage(5)) {}

    out_t Calculate(in_t pv, units::second_t dt, out_t feedforward = out_t{0}) {
      auto error = setpoint - pv;
      _integralSum += pv * dt;
      auto deriv = pv / dt;

      _stablePos = _posFilter.Calculate(error);
      _stableVel = _velFilter.Calculate(deriv);

      auto out = config.kp * error + config.ki * _integralSum + config.kd * deriv + feedforward;
      return out;
    }

    bool IsStable() const {
      return std::abs(_stablePos.value()) <= config.stableThresh.value()
        && std::abs(_stableVel.value()) <= config.stableDerivThresh.value();
    }

   private:
    units::unit_t<units::compound_unit<IN, units::second>> _integralSum;
    
    frc::LinearFilter<typename config_t::error_t> _posFilter;
    frc::LinearFilter<typename config_t::deriv_t> _velFilter;

    typename config_t::error_t _stablePos;
    typename config_t::deriv_t _stableVel;
  };
}