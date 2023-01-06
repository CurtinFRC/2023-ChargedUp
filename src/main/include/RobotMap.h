#pragma once

#include "Intake.h"
#include "VoltageController.h"
#include "DCMotor.h"
#include <frc/XboxController.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <Encoder.h>

struct RobotMap {
    struct Controllers {
        frc::XboxController
    driver{0};
    };
    Controllers controllers;
    struct IntakeSystem {
        wom::MotorVoltageController controller{new WPI_TalonSRX(10)};
        wom::Gearbox gearbox{
            &controller,
            nullptr,
            wom::DCMotor::Bag(1)
        };

        
    frc::DigitalInput sensor{0};

    IntakeConfig config{
        gearbox,
        &sensor
    };
    };

    // struct ArmSystem {
    //     wom::MotorVoltageController controller{new WPI_TalonSRX(999)};
    //     wom::Gearbox gearbox{
    //         &controller,
    //         nullptr,
    //         wom::DCMotor::Bag(1)
    //     };
    //     wml::sensors::Encoder;
        
    // frc::DigitalInput sensor{0};

    // IntakeConfig config{
    //     gearbox,
    //     &sensor
    // };
    // };
  
};