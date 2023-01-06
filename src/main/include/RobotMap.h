#pragma once

#include "Intake.h"
#include "Drivebase.h"
#include "VoltageController.h"
#include "DCMotor.h"
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>

struct RobotMap {
  struct Controllers {
    frc::XboxController driver{0};
  };
  Controllers controllers;

  struct MecanumDriveSystem { // WPI_TalonSRX(10) what is 10, and wot should i set it to
    wom::MotorVoltageController flMotorController{new WPI_TalonSRX(10)};
    wom::MotorVoltageController frMotorController{new WPI_TalonSRX(10)};
    wom::MotorVoltageController rlMotorController{new WPI_TalonSRX(10)};
    wom::MotorVoltageController rrMotorController{new WPI_TalonSRX(10)};

    wom::Gearbox frontLeftGearbox{
      &flMotorController,
      nullptr,
      wom::DCMotor::CIM(2).WithReduction(10.71)
    };
    wom::Gearbox frontRightGearbox{
      &frMotorController,
      nullptr,
      wom::DCMotor::CIM(2).WithReduction(8.45)
    };
    wom::Gearbox rearLeftGearbox{
      &rlMotorController,
      nullptr,
      wom::DCMotor::CIM(2).WithReduction(10.71)
    };
    wom::Gearbox rearRightGearbox{
      &rrMotorController,
      nullptr,
      wom::DCMotor::CIM(2).WithReduction(10.71)
    };
    
    MecanumDrivebaseConfig config{
      frontLeftGearbox, frontRightGearbox,
      rearLeftGearbox, rearRightGearbox,
      4_in / 2,
      frc::Translation2d{281_mm, 305_mm}, frc::Translation2d{281_mm, -305_mm},
      frc::Translation2d{-281_m, 305_m}, frc::Translation2d{-281_m, -305_m}
    };
  };
  MecanumDriveSystem mecanumDriveSystem;



  /**
   * Resources and Paramters related to the Intake Subsystem
   */
  struct IntakeSystem {
    /* Create a new Voltage Controller */
    wom::MotorVoltageController controller{new WPI_TalonSRX(10)};
    
    wom::Gearbox gearbox{
      &controller,
      nullptr,  /* nullptr for encoder means we have no encoder */
      wom::DCMotor::Bag(1).WithReduction(1) /* TODO: Update this reduction */
    };

    /* The beambreak sensor on Digital Channel 0 */
    frc::DigitalInput sensor{0};

    /* Create the intake config with the given resources */
    IntakeConfig config{
      gearbox,
      &sensor
    };
  };
  IntakeSystem intake;
  
};