#pragma once

#include "Intake.h"
#include "VoltageController.h"
#include "DCMotor.h"
#include "Arm.h"
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>

#include "sim/FakeEncoder.h"
#include "sim/FakeVoltageController.h"

struct RobotMap {
  struct Controllers {
    frc::XboxController driver{0};
  };
  Controllers controllers;

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
  

  struct ArmSystem {
    wom::FakeVoltageController controller;
    wom::FakeEncoder encoder{2048};

    wom::Gearbox gearbox{
      &controller,
      &encoder,
      wom::DCMotor::CIM(3).WithReduction(60)
    };

    frc::DigitalInput limitSwitch{1};

    wom::PIDConfig<units::radian, units::volt> pidConfig{
      "arm/pid/config",
      12_V / 20_deg, 
      0.2_V / (1_deg * 1_s)
    }; 

    ArmConfig config{
      gearbox,
      &limitSwitch,
      pidConfig
    };
  };
  ArmSystem arm;
};