#pragma once

#include "DCMotor.h"

#include <Eigen/Core>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <units/torque.h>
#include <units/velocity.h>

// Assuming all wheels have been calibrated to have forward @ +ve voltage.
const Eigen::Vector2d MECANUM_FL_BASIS{ 0.707, -0.707 };
const Eigen::Vector2d MECANUM_FR_BASIS{ 0.707, 0.707 };
const Eigen::Vector2d OMNI_REAR_BASIS{ 0, -1 };   // Right = +ve voltage

class HolonomicWheelSim {
 public:
  HolonomicWheelSim(wom::DCMotor motor, units::meter_t radius, units::kilogram_t mass, Eigen::Vector2d forceBasisVector) : 
    _motor(motor), _radius(radius), _mass(mass), _forceBasisVector(forceBasisVector.normalized()) {}

  Eigen::Vector2d Update(units::volt_t voltage, units::second_t dt) {
    auto torque = _motor.Torque(_motor.Current(1_rad * speed / _radius, voltage));
    _force = torque.value() / _radius.value() * _forceBasisVector;
    auto acceleration = torque / (_mass * _radius);
    speed += acceleration * dt;
    return _force;
  }

  units::meters_per_second_t speed{0};
 private:
  wom::DCMotor _motor;
  units::meter_t _radius;
  units::kilogram_t _mass;
  Eigen::Vector2d _forceBasisVector;

  Eigen::Vector2d _force;
};

class WASPSim {
 public:
  WASPSim(
    wom::DCMotor mecanumMotor, units::meter_t mecanumRadius,
    units::meter_t trackWidth,      // Width between mechanum wheels
    wom::DCMotor omniMotor, units::meter_t omniRadius,
    units::meter_t trackLength,     // Distance between mechanum wheels and omni wheels
    units::kilogram_square_meter_t J,
    units::kilogram_t mass
  ) : leftMecanum(mecanumMotor, mecanumRadius, mass / 3, MECANUM_FL_BASIS),
      rightMecanum(mecanumMotor, mecanumRadius, mass / 3, MECANUM_FR_BASIS),
      rearOmni(omniMotor, omniRadius, mass / 3, OMNI_REAR_BASIS),
      _J(J), _mass(mass), _trackWidth(trackWidth), _trackLength(trackLength)
  { }

  void Update(units::volt_t leftVoltage, units::volt_t rightVoltage, units::volt_t rearVoltage, units::second_t dt) {
    Eigen::Matrix<double, 2, 3> forces;
    forces.block<2, 1>(0, 0) = leftMecanum.Update(leftVoltage, dt);
    forces.block<2, 1>(0, 1) = rightMecanum.Update(rightVoltage, dt);
    forces.block<2, 1>(0, 2) = rearOmni.Update(rearVoltage, dt);

    Eigen::Vector2d netForce = forces.rowwise().sum();

    Eigen::Vector3d torque{
      forces(0, 0) * -_trackWidth.value() / 2,
      forces(0, 1) * _trackWidth.value() / 2,
      forces(1, 2) * -_trackLength.value()
    };

    units::radians_per_second_squared_t angular_accel = 1_rad * units::newton_meter_t{torque.sum()} / _J;
    omega += angular_accel * dt;
    theta += omega * dt;

    Eigen::Vector2d accel = netForce / _mass.value();

    vx += units::meters_per_second_squared_t{accel(0)} * dt;
    vy += units::meters_per_second_squared_t{accel(1)} * dt;

    x += vx * dt;
    y += vy * dt;
  }

  units::radians_per_second_t omega{0};
  units::radian_t theta{0};

  units::meters_per_second_t vx{0}, vy{0};
  units::meter_t x{0}, y{0};

  HolonomicWheelSim leftMecanum, rightMecanum;
  HolonomicWheelSim rearOmni;

 private:
  units::kilogram_square_meter_t _J;
  units::kilogram_t _mass;
  units::meter_t _trackWidth, _trackLength;
};
