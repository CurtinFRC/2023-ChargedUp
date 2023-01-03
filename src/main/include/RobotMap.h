#pragma once

#include <rev/CANSparkMax.h>
#include <frc/system/plant/DCMotor.h>
#include "Encoder.h"
#include "Gearbox.h"
#include "PID.h"
#include "drivetrain/Drivetrain.h"
#include "AHRS.h"

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

  struct DrivetrainSystem {
    double GEARBOX_REDUCTION = 1.0 / (14.0 * 14.0 / 32.0 / 36.0);

    WPI_TalonFX leftDriveMotor{6};
    WPI_TalonFX rightDriveMotor{7};

    wom::TalonFXEncoder leftDriveEncoder{&leftDriveMotor, GEARBOX_REDUCTION};
    wom::TalonFXEncoder rightDriveEncoder{&rightDriveMotor, GEARBOX_REDUCTION};

    wom::Gearbox leftDrive {
      new wom::MotorVoltageController(&leftDriveMotor),
      &leftDriveEncoder,
      wom::DCMotor::Falcon500(1).WithReduction(GEARBOX_REDUCTION)
    };

    wom::Gearbox rightDrive {
      new wom::MotorVoltageController(&rightDriveMotor),
      &rightDriveEncoder,
      wom::DCMotor::Falcon500(1).WithReduction(GEARBOX_REDUCTION)
    };

    AHRS gyro{frc::SPI::Port::kMXP};

    wom::PIDConfig<units::meters_per_second, units::volt> velocityPID {
      "drivetrain/pid/velocity/config",
      12_V / 2_mps
    };

    wom::PIDConfig<units::meter, units::meters_per_second> distancePID {
      "drivetrain/behaviours/DrivetrainDriveDistance/pid/config",
      4_mps / 1_m,
      // 0_mps / 1_s / 1_m,
      // 0_m / 1_m,
      // 5_cm,
      // 0.2_mps
    };

    wom::PIDConfig<units::degree, units::degrees_per_second> anglePID {
      "drivetrain/behaviours/DrivetrainTurnAngle/pid/config",
      (180_deg / 0.5_s) / 45_deg,
      // 0_deg / 1_s / 1_s / 1_deg,
      // 0_deg / 1_deg,
      // 2_deg,
      // 1_deg / 1_s
    };

    wom::DrivetrainConfig config{
      leftDrive,
      rightDrive,

      &gyro,

      4_in / 2,
      0.54_m,
      
      velocityPID,
      distancePID,
      anglePID
    };
    
  }; DrivetrainSystem drivetrain;

};