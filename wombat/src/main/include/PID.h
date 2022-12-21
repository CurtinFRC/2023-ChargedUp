#pragma once

#include <units/base.h>
#include <units/time.h>

#include <frc/filter/LinearFilter.h>

#include <optional>

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
    deriv_t stableDerivThresh{-1};

    units::unit_t<IN> izone{-1};
  };

  template<typename IN, typename OUT>
  class PIDController {
   public:
    using config_t = PIDConfig<IN, OUT>;
    using in_t = units::unit_t<IN>;
    using out_t = units::unit_t<OUT>;
    using sum_t = units::unit_t<units::compound_unit<IN, units::second>>;

    config_t config;

    PIDController(config_t initialGains, in_t setpoint = in_t{0}) 
      : config(initialGains), _setpoint(setpoint),
        _posFilter(frc::LinearFilter<typename config_t::error_t>::MovingAverage(20)),
        _velFilter(frc::LinearFilter<typename config_t::deriv_t>::MovingAverage(20)) {}

    void SetSetpoint(in_t setpoint) {
      if (std::abs(setpoint.value() - _setpoint.value()) > 0.05 * _setpoint.value()) {
        _iterations = 0;
      }
      _setpoint = setpoint;
    }

    in_t GetSetpoint() const {
      return _setpoint;
    }

    void SetWrap(std::optional<in_t> range) {
      _wrap_range = range;
    }

    out_t Calculate(in_t pv, units::second_t dt, out_t feedforward = out_t{0}) {
      auto error = do_wrap(_setpoint - pv);
      _integralSum += error * dt;
      if (config.izone.value() > 0 && (error > config.izone || error < -config.izone))
        _integralSum = sum_t{0};
      
      typename config_t::deriv_t deriv{0};

      if (_iterations > 0)
        deriv = (pv - _last_pv) / dt;

      _stablePos = _posFilter.Calculate(error);
      _stableVel = _velFilter.Calculate(deriv);

      auto out = config.kp * error + config.ki * _integralSum + config.kd * deriv + feedforward;

      _last_pv = pv;
      _iterations++;
      return out;
    }

    bool IsStable() const {
      return _iterations > 20
        && std::abs(_stablePos.value()) <= config.stableThresh.value()
        && (config.stableDerivThresh.value() < 0 || std::abs(_stableVel.value()) <= config.stableDerivThresh.value());
    }

   private:
    in_t do_wrap(in_t val) {
      if (_wrap_range.has_value()) {
        double wr = _wrap_range.value().value();
        double v = val.value();

        v = std::fmod(v, wr);
        if (std::abs(v) > (wr / 2.0)) {
          return in_t{(v > 0) ? v - wr : v + wr};
        } else {
          return in_t{v};
        }
      }
      return val;
    }

    sum_t _integralSum;
    in_t _setpoint;
    in_t _last_pv{0};

    std::optional<in_t> _wrap_range;
    
    int _iterations = 0;

    frc::LinearFilter<typename config_t::error_t> _posFilter;
    frc::LinearFilter<typename config_t::deriv_t> _velFilter;

    typename config_t::error_t _stablePos;
    typename config_t::deriv_t _stableVel;
  };
}