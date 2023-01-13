#pragma once

#include "VoltageController.h"
#include "Arm.h"
#include "Gyro.h"

#include <frc/XboxController.h>
#include <ctre/Phoenix.h>

#include "drivetrain/SwerveDrive.h"

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
      wom::DCMotor::CIM(2).WithReduction(100)
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

  struct SwerveBase{
    wom::NavX gyro;
    wpi::array<WPI_TalonFX*, 4> turnMotors{
      new WPI_TalonFX(1), new WPI_TalonFX(2), new WPI_TalonFX(3), new WPI_TalonFX(4)
    };
    wpi::array<WPI_TalonFX*, 4> driveMotors{
      new WPI_TalonFX(5), new WPI_TalonFX(6), new WPI_TalonFX(7), new WPI_TalonFX(8)
    };
    
    wpi::array<wom::SwerveModuleConfig, 4> moduleConfigs{
      wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
        frc::Translation2d(0.5_m, 0.5_m),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[0]),
          new wom::TalonFXEncoder(driveMotors[0]),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[0]),
          new wom::TalonFXEncoder(turnMotors[0]),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
      wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
        frc::Translation2d(0.5_m, -0.5_m),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[1]),
          new wom::TalonFXEncoder(driveMotors[1]),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[1]),
          new wom::TalonFXEncoder(turnMotors[1]),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
      wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
        frc::Translation2d(-0.5_m, 0.5_m),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[2]),
          new wom::TalonFXEncoder(driveMotors[2]),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[2]),
          new wom::TalonFXEncoder(turnMotors[2]),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
      wom::SwerveModuleConfig{ // dimensions are assuming perfect square robot 1m^2 area
        frc::Translation2d(-0.5_m, -0.5_m),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[3]),
          new wom::TalonFXEncoder(driveMotors[3]),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[3]),
          new wom::TalonFXEncoder(turnMotors[3]),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
    };

    wom::SwerveModule::angle_pid_conf_t anglePID {
      "/drivetrain/pid/angle/config",
      12_V / 90_deg
    };
    wom::SwerveModule::velocity_pid_conf_t velocityPID{
      "/drivetrain/pid/velocity/config",
      12_V / 2_mps
    };
    wom::SwerveDriveConfig config{
      "/drivetrain",
      anglePID, velocityPID,
      moduleConfigs,// each module
      &gyro,
      {
        "/drivetrain/pid/pose/angle/config",
        180_deg / 1_s / 45_deg
      },
      {
        "/drivetrain/pid/pose/position/config",
        3_mps / 1_m
      },
      70_kg // robot mass (estimate rn)
    };  
  };
  SwerveBase swerveBase;

};