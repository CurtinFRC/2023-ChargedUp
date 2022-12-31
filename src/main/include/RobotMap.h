#pragma once

#include <rev/CANSparkMax.h>
#include <frc/system/plant/DCMotor.h>
#include "Encoder.h"
#include "Gearbox.h"
#include "PID.h"

struct RobotMap {

  struct ShooterSystem {
    rev::CANSparkMax leftFlyWheelMotor{ 11, rev::CANSparkMax::MotorType::kBrushless };
    rev::CANSparkMax rightFlyWheelMotor{ 12, rev::CANSparkMax::MotorType::kBrushless };
    rev::CANSparkMax centerFlyWheelMotor{ 13, rev::CANSparkMax::MotorType::kBrushless };

    wom::MotorVoltageController shooterMotorGroup = wom::MotorVoltageController::Group(
      wom::invert(leftFlyWheelMotor),
      wom::invert(rightFlyWheelMotor),
      wom::invert(centerFlyWheelMotor)
    );

    wom::CANSparkMaxEncoder encoder{&leftFlyWheelMotor};

    wom::Gearbox shooterGearbox {
      &shooterMotorGroup,
      &encoder,
      wom::DCMotor::NEO(3)
    };

    wom::PIDConfig<units::radians_per_second, units::volt> pid{
      "shooter/pid/config",
      12_V / 3500_rpm,
      0.02_V / (2000_rpm * 1_s),
      0_V / (5000_rpm / 1_s),

      50_rpm,
      25_rpm / 1_s
    };

    units::ampere_t currentLimit{35};
  }; ShooterSystem shooter;

};