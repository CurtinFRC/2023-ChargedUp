#pragma once

#include "VoltageController.h"
#include "Arm.h"

#include <frc/XboxController.h>
#include <ctre/Phoenix.h>

struct RobotMap {
  struct Controllers {
    frc::XboxController driver{0};
    frc::XboxController codriver{1};
  };
  Controllers controllers;

  struct Arm {

    WPI_TalonSRX motor1{1};
    WPI_TalonSRX motor2{2};

    wom::MotorVoltageController motor = wom::MotorVoltageController::Group(motor1, motor2);

    wom::DigitalEncoder encoder{0, 1, 2048};

    wom::Gearbox gearbox {
      &motor,
      &encoder,
      wom::DCMotor::CIM(2).WithReduction(50)
    };
    wom::ArmConfig config {
      "/arm",
      gearbox,
      nullptr,
      nullptr,
      {
        "/arm/pid/config",
        12_V / 45_deg
      },
      5_kg, 10_kg, 1_m
    };
  };
  Arm arm;
};