#pragma once

#include <iostream>
#include <string>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/Compressor.h>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>
#include <units/length.h>
#include <frc/XboxController.h>
#include <units/angle.h>

#include "Encoder.h"
#include "drivetrain/SwerveDrive.h"
#include "VoltageController.h"
#include "Gyro.h"
#include "Vision.h"
#include "XInputController.h"
#include "behaviour/VisionBehaviour.h"

#include "Arm.h"
#include "Elevator.h"
#include "Armavator.h"
#include "Gripper.h"
#include "TOF.h"
#include "behaviour/ArmavatorBehaviour.h"


struct RobotMap {
  struct Controllers {    
    //sets driver station numbers for the controllers
    frc::XboxController driver{0};
    frc::XboxController codriver{1};
    frc::XboxController test{2};
  };
  Controllers controllers;

  struct ControlSystem {
    frc::Compressor pcmCompressor{2, frc::PneumaticsModuleType::REVPH};
  }; ControlSystem controlSystem;

  struct GripperSystem {
    wom::MotorVoltageController gripperMotor{ new WPI_VictorSPX(15)};

    TOF gamepiecePresence{frc::I2C::Port::kMXP};

    GripperConfig config{
      &gripperMotor
    };
  }; GripperSystem gripper;

  struct Vision {
    VisionConfig config{
      std::make_shared<photonlib::PhotonCamera>("camera"), 
      frc::Transform3d{ frc::Translation3d{ 0_m, 0_m, 0_m }, frc::Rotation3d{ 0_rad, 0_rad, 0_rad } },
      70_deg,
      Get2023Layout()
    };
  }; Vision vision;

  struct SwerveBase{
    CANCoder frontLeftCancoder{19};
    CANCoder frontRightCancoder{17};
    CANCoder backLeftCancoder{16};
    CANCoder backRightCancoder{18};

    wom::NavX gyro;
    wpi::array<WPI_TalonFX*, 4> turnMotors{
      new WPI_TalonFX(1), new WPI_TalonFX(3), new WPI_TalonFX(5), new WPI_TalonFX(7)
    };
    wpi::array<WPI_TalonFX*, 4> driveMotors{
      new WPI_TalonFX(2), new WPI_TalonFX(4), new WPI_TalonFX(6), new WPI_TalonFX(8)
    };

    wpi::array<wom::SwerveModuleConfig, 4> moduleConfigs{
      wom::SwerveModuleConfig{ // front left module
        frc::Translation2d(10.761_in, 9.455_in),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[0]),
          new wom::TalonFXEncoder(driveMotors[0], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[0]),
          new wom::CanEncoder(19, 4096, 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        &frontLeftCancoder,
        4_in / 2
      },
      wom::SwerveModuleConfig{ // front right module
        frc::Translation2d(10.761_in, -9.455_in),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[1]),
          new wom::TalonFXEncoder(driveMotors[1], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[1]),
          new wom::CanEncoder(17, 4096, 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        &frontRightCancoder,
        4_in / 2
      },
      wom::SwerveModuleConfig{ // back left module
        frc::Translation2d(-10.761_in, 9.455_in),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[2]),
          new wom::TalonFXEncoder(driveMotors[2], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[2]),
          new wom::CanEncoder(16, 4096, 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        &backRightCancoder,
        4_in / 2
      },
      wom::SwerveModuleConfig{ // back right module
        frc::Translation2d(-10.761_in, -9.455_in),
        wom::Gearbox{
          new wom::MotorVoltageController(driveMotors[3]),
          new wom::TalonFXEncoder(driveMotors[3], 6.75),
          wom::DCMotor::Falcon500(1).WithReduction(6.75)
        },
        wom::Gearbox{
          new wom::MotorVoltageController(turnMotors[3]),
          new wom::CanEncoder(18, 4096, 12.8),
          wom::DCMotor::Falcon500(1).WithReduction(12.8)
        },
        &backLeftCancoder,
        4_in / 2
      },
    };

    // Setting the PID path and values to be used for SwerveDrive and SwerveModules
    wom::SwerveModule::angle_pid_conf_t anglePID {
      "/drivetrain/pid/angle/config",
      14_V / 180_deg,
      0.0_V / (100_deg * 1_s),
      0_V / (100_deg / 1_s),
      1_deg,
      0.5_deg / 2_s
    };
    wom::SwerveModule::velocity_pid_conf_t velocityPID{
      "/drivetrain/pid/velocity/config",
      //  12_V / 4_mps
    };
    wom::SwerveDriveConfig::pose_angle_conf_t poseAnglePID {
      "/drivetrain/pid/pose/angle/config",
      180_deg / 1_s / 45_deg,
      wom::SwerveDriveConfig::pose_angle_conf_t::ki_t{0.1},
      0_deg / 1_deg,
      10_deg,
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

    wom::SwerveDriveConfig config{
      "/drivetrain",
      anglePID, velocityPID,
      moduleConfigs,// each module
      &gyro,
      poseAnglePID, 
      posePositionPID,
      60_kg, // robot mass (estimate is kinda good enough), keep in mind this needs to be total weight, not the weight recorded in inspection as this doesnt include bumpers or battery weight
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


  struct Armavator {
    //sets up the percieved masses for the load, arm and carraige
    static constexpr units::kilogram_t loadMass = 1_kg;
    static constexpr units::kilogram_t armMass = 2_kg;
    static constexpr units::kilogram_t carriageMass = 3_kg;

    //stores nessesary info for arm
    struct Arm {
      //creates the motor used for the arm as well as the port it is plugged in
      rev::CANSparkMax leftArmMotor{11, rev::CANSparkMax::MotorType::kBrushless}; //11
      rev::CANSparkMax rightArmMotor{12, rev::CANSparkMax::MotorType::kBrushless}; //12

      rev::CANSparkMax leftPretendArmMotor{28, rev::CANSparkMax::MotorType::kBrushless};
      rev::CANSparkMax rightPretendArmMotor{29, rev::CANSparkMax::MotorType::kBrushless};

      //create the motor group used for the arm
      wom::MotorVoltageController leftMotorGroup = wom::MotorVoltageController::Group(leftArmMotor);
      wom::MotorVoltageController rightMotorGroup = wom::MotorVoltageController::Group(rightArmMotor);
      
      // wom::DigitalEncoder encoder{0, 1, 2048};
      //sets the type sof encoder that is used up
      wom::CANSparkMaxEncoder leftEncoder{&leftPretendArmMotor, 100};
      wom::CANSparkMaxEncoder rightEncoder{&rightPretendArmMotor, 100};

      // wom::CANSparkMaxEncoder leftEncoder{&leftArmMotor, 100};
      // wom::CANSparkMaxEncoder rightEncoder{&rightArmMotor, 100};

      rev::SparkMaxRelativeEncoder leftOtherArmEncoder = leftArmMotor.GetEncoder();
      rev::SparkMaxRelativeEncoder rightOtherArmEncoder = rightArmMotor.GetEncoder();

      //creates an instance of the arm gearbox
      wom::Gearbox leftGearbox {
        &leftMotorGroup,
        &leftEncoder,
        // nullptr,
        wom::DCMotor::NEO(1).WithReduction(100)
      };

      wom::Gearbox rightGearbox {
        &rightMotorGroup,
        &rightEncoder,
        // nullptr,
        wom::DCMotor::NEO(1).WithReduction(100)
      };

      //creates arm config information
      wom::ArmConfig config {
        "/armavator/arm",
        leftGearbox,
        rightGearbox,
        leftOtherArmEncoder,
        wom::PIDConfig<units::radian, units::volts>(
          "/armavator/arm/pid/config",
          13_V / 25_deg, //prev 13_V/25_deg
          0.1_V / (1_deg * 1_s), //0.1_V / (1_deg * 1_s)
          0_V / (1_deg / 1_s),
          5_deg,
          2_deg / 1_s,
          10_deg
        ),
        wom::PIDConfig<units::radians_per_second, units::volts>(
          "/armavator/arm/velocityPID/config",
          9_V / (180_deg / 1_s),
          0_V / 25_deg,
          0_V / (90_deg / 1_s / 1_s)
        ),
        2_kg, 
        2_kg,
        1.37_m,
        -90_deg,
        270_deg,
        0_deg
      };

      Arm() {
        //inverts the motor so that it goes in the right direction while using RAW controlls
        leftArmMotor.SetInverted(true);
        rightArmMotor.SetInverted(false);
      }
    };
    Arm arm;

    struct Elevator {
      //creates instances of the motors used for the elevator as well as what ports they are plugged in to
      rev::CANSparkMax leftElevatorMotor{9, rev::CANSparkMax::MotorType::kBrushless}; //9
      rev::CANSparkMax rightElevatorMotor{10, rev::CANSparkMax::MotorType::kBrushless}; //10

      rev::CANSparkMax leftPretendElevatorMotor{25, rev::CANSparkMax::MotorType::kBrushless}; //10
      rev::CANSparkMax rightPretendElevatorMotor{26, rev::CANSparkMax::MotorType::kBrushless}; //10

      //creates the motor group that can be used to set voltage
      wom::MotorVoltageController leftMotorGroup = wom::MotorVoltageController::Group(leftElevatorMotor);
      wom::MotorVoltageController rightMotorGroup = wom::MotorVoltageController::Group(rightElevatorMotor);

      //creates an instance of the encoder that will be used for the elevator
      wom::CANSparkMaxEncoder leftEncoder{&leftPretendElevatorMotor, 14/60};
      wom::CANSparkMaxEncoder rightEncoder{&rightPretendElevatorMotor, 14/60};

      rev::SparkMaxRelativeEncoder leftOtherEncoder= leftElevatorMotor.GetEncoder();
      rev::SparkMaxRelativeEncoder rightOtherEncoder= rightElevatorMotor.GetEncoder();

      //creates an instance of the gearbox used for the elevator
      wom::Gearbox leftGearbox {
        &leftMotorGroup,
        &leftEncoder,
        // nullptr,
        wom::DCMotor::NEO(1).WithReduction(14/60)
      };

      wom::Gearbox rightGearbox {
        &rightMotorGroup,
        &rightEncoder,
        // nullptr,
        wom::DCMotor::NEO(1).WithReduction(14/60)
      };

      //creates the elevator config information to use
      wom::ElevatorConfig config {
        "/armavator/elevator",
        leftGearbox,
        rightGearbox,
        leftOtherEncoder,
        nullptr,
        nullptr,
        48.26_mm / 2,
        armMass + loadMass + carriageMass,
        1_m,
        0_m,
        0_m,
        {
          //creates the pid for the elevator to remove error
          "/armavator/elevator/pid/config",
          20_V / 1_m, //16V
          0.3_V / (1_m * 1_s),
          0_V / (1_m / 1_s),
          0.1_m,
          0.05_m / 1_s
        },
        {
          "/armavator/elevator/velocity/pid/config",
          6_V / (1_m / 1_s),
          0_V / 1_m,
          0_V / (1_m / 1_s / 1_s)
        }
      };

      //inverts the motor directions so that the arm goes to the right place during RAW control
      Elevator() {
        leftElevatorMotor.SetInverted(false);
        rightElevatorMotor.SetInverted(true);
      }
    };
    Elevator elevator;

    //creates the config for the occupancygrid 
    ArmavatorConfig::grid_t occupancyGrid = ArmavatorConfig::grid_t(
      arm.config.minAngle, arm.config.maxAngle,
      0_m, elevator.config.maxHeight,
      50, 50
    ).FillF([this](units::radian_t angle, units::meter_t height) {
      units::meter_t x = arm.config.armLength * units::math::cos(angle);
      units::meter_t y = height + arm.config.armLength * units::math::sin(angle);
      return !(y >= 0.1_m && y <= 6_ft);
    });

    ArmavatorConfig config {
      arm.config, elevator.config, occupancyGrid
    };
  }; Armavator armavator;

  //creates the arm and swerve instances for network tables on shuffleboard
  struct ArmTable {
    std::shared_ptr<nt::NetworkTable> armManualTable = nt::NetworkTableInstance::GetDefault().GetTable("armManual");
  }; ArmTable armTable;

  struct SwerveTable {
    std::shared_ptr<nt::NetworkTable> swerveDriveTable = nt::NetworkTableInstance::GetDefault().GetTable("swerve");
  }; SwerveTable swerveTable;

};