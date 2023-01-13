#pragma once

#include "VoltageController.h"
#include "Arm.h"
#include "Elevator.h"
#include "Armavator.h"

#include <frc/XboxController.h>
#include <ctre/Phoenix.h>

struct RobotMap {
  struct Controllers {
    frc::XboxController driver{0};
    frc::XboxController codriver{1};
  };
  Controllers controllers;

  struct Armavator {
    static constexpr units::kilogram_t loadMass = 10_kg;
    static constexpr units::kilogram_t armMass = 5_kg;
    static constexpr units::kilogram_t carriageMass = 5_kg;

    struct Arm {
      WPI_TalonSRX motor1{1};
      WPI_TalonSRX motor2{2};

      wom::MotorVoltageController motor = wom::MotorVoltageController::Group(motor1, motor2);

      wom::DigitalEncoder encoder{0, 1, 2048};

      wom::Gearbox gearbox {
        &motor,
        &encoder,
        wom::DCMotor::CIM(2).WithReduction(100)
      };
      wom::ArmConfig config {
        "/armavator/arm",
        gearbox,
        nullptr,
        nullptr,
        {
          "/armavator/arm/pid/config",
          12_V / 45_deg
        },
        armMass, loadMass, 1_m,
        -90_deg, 270_deg,
        -90_deg
      };
    };
    Arm arm;

    struct Elevator {
      WPI_TalonSRX motor1{3};
      WPI_TalonSRX motor2{4};

      wom::MotorVoltageController motor = wom::MotorVoltageController::Group(motor1, motor2);

      wom::DigitalEncoder encoder{2, 3, 2048};

      wom::Gearbox gearbox {
        &motor,
        &encoder,
        wom::DCMotor::CIM(2).WithReduction(10)
      };

      wom::ElevatorConfig config {
        "/armavator/elevator",
        gearbox,
        nullptr,
        nullptr,
        2_in,
        armMass + loadMass + carriageMass,
        1.5_m,
        {
          "/armavator/elevator/pid/config",
          12_V / 1_m
        }
      };
    };
    Elevator elevator;

    ArmavatorConfig config {
      arm.config, elevator.config
    };
  };
  Armavator armavator;
};