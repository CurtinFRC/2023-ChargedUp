#pragma once
#include "VoltageController.h"
#include "Gyro.h"
#include "Vision.h"

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/Compressor.h>

#include "XInputController.h"

#include <ctre/Phoenix.h>

#include "drivetrain/SwerveDrive.h"
#include <frc/DoubleSolenoid.h>
#include <units/length.h>

// #include "DriverController.h"

#include <iostream>
#include <string>

struct RobotMap {

  struct Controllers {    
    //sets driver station numbers for the controllers
    wom::PS4Controller driver{0};
    wom::XboxController codriver{1};
  };
  Controllers controllers;

  //stores nessesary info for vision
  struct Vision {
    VisionConfig config{
      std::make_shared<photonlib::PhotonCamera>("camera"), 
      frc::Transform3d{ frc::Translation3d{ 0_m, 0_m, 0_m }, frc::Rotation3d{ 0_rad, 0_rad, 0_rad } },
      Get2023Layout()
    };
  };
  Vision vision;

  //stores nessesary info for swerve
  struct SwerveBase{
    wom::NavX gyro;
    wpi::array<WPI_TalonFX*, 4> turnMotors{
      new WPI_TalonFX(6), new WPI_TalonFX(4), new WPI_TalonFX(3), new WPI_TalonFX(1)
    };
    wpi::array<WPI_TalonFX*, 4> driveMotors{
      new WPI_TalonFX(7), new WPI_TalonFX(2), new WPI_TalonFX(8), new WPI_TalonFX(5)
    };    
    wpi::array<wom::SwerveModuleConfig, 4> moduleConfigs{ 
      wom::SwerveModuleConfig{ // front left module
        frc::Translation2d(10.761_in, 9.455_in),
        ctre::phoenix::sensors::CANCoder(99),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[0]),
          new wom::TalonFXEncoder(driveMotors[0], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[0]),
          new wom::TalonFXEncoder(turnMotors[0], 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
      wom::SwerveModuleConfig{ // front right module
        frc::Translation2d(10.761_in, -9.455_in),
        ctre::phoenix::sensors::CANCoder(99),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[1]),
          new wom::TalonFXEncoder(driveMotors[1], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[1]),
          new wom::TalonFXEncoder(turnMotors[1], 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
      wom::SwerveModuleConfig{ // back left module
        frc::Translation2d(-10.761_in, 9.455_in),
        ctre::phoenix::sensors::CANCoder(99),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[2]),
          new wom::TalonFXEncoder(driveMotors[2], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[2]),
          new wom::TalonFXEncoder(turnMotors[2], 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
      wom::SwerveModuleConfig{ // back right module
        frc::Translation2d(-10.761_in, -9.455_in),
        ctre::phoenix::sensors::CANCoder(99),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[3]),
          new wom::TalonFXEncoder(driveMotors[3], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[3]),
          new wom::TalonFXEncoder(turnMotors[3], 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        4_in / 2
      },
    };

    // Setting the PID path and values to be used for SwerveDrive and SwerveModules
    wom::SwerveModule::angle_pid_conf_t anglePID {
      "/drivetrain/pid/angle/config",
      10.5_V / 180_deg,
      0.75_V / (100_deg * 1_s),
      0_V / (100_deg / 1_s),
      1_deg,
      0.5_deg / 1_s
    };
    wom::SwerveModule::velocity_pid_conf_t velocityPID{
      "/drivetrain/pid/velocity/config",
      //  12_V / 4_mps // webers per metre
    };
    wom::SwerveDriveConfig::pose_angle_conf_t poseAnglePID {
      "/drivetrain/pid/pose/angle/config",
      180_deg / 1_s / 45_deg,
      wom::SwerveDriveConfig::pose_angle_conf_t::ki_t{0.1},
      0_deg / 1_deg,
      3_deg,
      10_deg / 1_s
    };
    wom::SwerveDriveConfig::pose_position_conf_t posePositionPID{
      "/drivetrain/pid/pose/position/config",
      3_mps / 1_m,
      wom::SwerveDriveConfig::pose_position_conf_t::ki_t{0.15},
      0_m / 1_m,
      20_cm, 
      10_cm / 1_s,
      10_cm
    };

    // the config for the whole swerve drive
    wom::SwerveDriveConfig config{
      "/drivetrain",
      anglePID, velocityPID,
      moduleConfigs,// each module
      &gyro,
      poseAnglePID, 
      posePositionPID,
      10_kg, // robot mass (estimate rn)
      {0.1, 0.1, 0.1},
      {0.9, 0.9, 0.9}
    };  

    // current limiting and setting idle mode of modules to brake mode
    SwerveBase() {
      for (size_t i = 0; i < 4; i++) {
        turnMotors[i]->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 15, 15, 0));
        driveMotors[i]->SetNeutralMode(NeutralMode::Brake);
        turnMotors[i]->SetNeutralMode(NeutralMode::Brake);
        driveMotors[i]->SetInverted(true);
      }
    }
  };
  SwerveBase swerveBase;

  struct SwerveGridPoses { // positions to place the items
    frc::Pose2d innerGrid1; // Closest grid position to the Wall
    frc::Pose2d innerGrid2; // Middle of Inner Grid
    frc::Pose2d innerGrid3; // Centremost Inner Grid position
    frc::Pose2d centreGrid1; // The non central grid on the Inner Grid side
    frc::Pose2d centreGrid2; // The middle most grid
    frc::Pose2d centreGrid3; // The non central grid on the Outer Grid side
    frc::Pose2d outerGrid1; // Centremost outer grid position
    frc::Pose2d outerGrid2; // Middle of Outer Grid
    frc::Pose2d outerGrid3; // Closest grid position to enemy Loading Zone
  };

  SwerveGridPoses bluePoses{ // grid poses for blue
    frc::Pose2d(20.185_in, 20.208_in, 0_deg),
    frc::Pose2d(42.2_in, 20.208_in, 0_deg),
    frc::Pose2d(64.185_in, 20.208_in, 0_deg),
    frc::Pose2d(86.078_in, 20.208_in, 0_deg),
    frc::Pose2d(108.131_in, 20.208_in, 216_deg),
    frc::Pose2d(130.185_in, 20.208_in, 0_deg),
    frc::Pose2d(152.185_in, 20.208_in, 0_deg),
    frc::Pose2d(174.170_in, 20.208_in, 0_deg),
    frc::Pose2d(196.185_in, 20.208_in, 0_deg)
  };
  // TO BE DONE BELOW!!!!!!!!!!!!!!!!!
  SwerveGridPoses redPoses{ // grid poses for red
    frc::Pose2d(20.185_in, 20.208_in, 0_deg),
    frc::Pose2d(42.2_in, 20.208_in, 0_deg),
    frc::Pose2d(64.185_in, 20.208_in, 0_deg),
    frc::Pose2d(86.078_in, 20.208_in, 0_deg),
    frc::Pose2d(108.131_in, 20.208_in, 216_deg),
    frc::Pose2d(130.185_in, 20.208_in, 0_deg),
    frc::Pose2d(152.185_in, 20.208_in, 0_deg),
    frc::Pose2d(174.170_in, 20.208_in, 0_deg),
    frc::Pose2d(196.185_in, 20.208_in, 0_deg)
  };

  SwerveGridPoses swerveGridPoses;
  
  struct SwerveTable {
    std::shared_ptr<nt::NetworkTable> swerveDriveTable = nt::NetworkTableInstance::GetDefault().GetTable("swerve");
  };
  SwerveTable swerveTable;
};